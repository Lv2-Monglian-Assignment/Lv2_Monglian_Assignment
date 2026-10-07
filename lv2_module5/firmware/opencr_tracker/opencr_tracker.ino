// 추적용 펌웨어: Pi(tracker_bridge)의 속도 명령을 받아 팬(ID 11)·틸트(ID 12)를 구동한다.
// XM430-W350, Protocol 2.0, 1 Mbps. 두 모터를 속도 모드(1)로 바꾼다(EEPROM, 한 번만 쓰임).
// 시리얼 프로토콜은 이슈 #7 제안을 따른다(115200 baud, ASCII 한 줄 = 한 메시지). 작업 이슈 #14.
//
//   Pi → OpenCR  V <pan_dps> <tilt_dps>   속도 명령 [°/s]
//                I                        IDLE 자세(두 축 0°)로 이동 후 정지
//                X                        즉시 정지 (속도 0, 토크 유지)
//                O                        토크 OFF  (#7에 없는 추가안)
// 토크는 0이 아닌 V 또는 I를 받을 때 켠다.
//   OpenCR → Pi  S <ms> <pan_deg> <tilt_deg> <pan_dps> <tilt_dps> <state>   50 Hz
//                E <code> <text>          오류·경고
//
// 제어 루프 100 Hz: 위치·속도 읽기 → 통신 타임아웃 검사 → 명령 선택(추종/IDLE 복귀 P) →
// 각도 제한(한계에 가까울수록 바깥 방향 속도를 줄임) → 속도 상한 → 속도 쓰기.
// 각도는 IDLE 기준 관절각이며 부호는 URDF와 같다: 팬 + = 왼쪽, 틸트 + = 위.
// IDLE(임시 정의): 팬 원시값 0°(0 tick), 틸트 원시값 180°(2048 tick). Homing Offset은 쓰지 않는다.
//
// 상태: OFF(토크 OFF) / HOLD(토크 ON, 속도 0) / TRACK(속도 추종) / HOMING(IDLE로 이동) / FAULT
// 보드 측 안전 정지:
//   - 유효한 명령이 300 ms 없으면 속도 0(HOLD), 5 s 더 없으면 토크 OFF
//   - 모터 Bus Watchdog 200 ms: OpenCR가 멈춰도 모터가 스스로 정지
//   - DXL 통신 실패·루프 지연 시 토크 OFF 후 FAULT (RESET 필요)
#include <Dynamixel2Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>

using namespace ControlTableItem;
Dynamixel2Arduino dxl(Serial3, 84);  // OpenCR DXL 포트 / 방향 제어 핀

const uint32_t DXL_BAUD = 1000000;
const uint8_t N = 2;
const uint8_t IDS[N] = {11, 12};
const int32_t IDLE_TICKS[N] = {0, 2048};           // 원시값 0°, 180°
const float LIMIT_DEG[N] = {180.0f, 40.0f};        // 소프트 한계 (둘 다 기구 여유를 고려한 값)
const float ENABLE_LIMIT_DEG[N] = {170.0f, 45.0f}; // 토크를 켤 때 IDLE에서 이보다 멀면 거부
const float SPEED_LIMIT_DPS = 120.0f;              // 펌웨어 속도 상한 (Kp 시험과 같은 값)
const float LIMIT_GAIN = 3.0f;                     // [1/s] 한계 접근 시 바깥 방향 허용 속도 = 3 × 남은 각도
const float HOME_KP = 2.0f;                        // [1/s] IDLE 복귀 위치 P (Kp 시험에서 고른 값)
const float HOME_SPEED_DPS = 30.0f;
const float HOME_TOL_DEG = 0.5f;
const uint32_t HOME_TIMEOUT_MS = 15000;

const uint32_t PERIOD_US = 10000;                  // 100 Hz 제어 루프
const uint8_t STATUS_DIV = 2;                      // 상태 회신 50 Hz
const uint32_t LATE_US = 5 * PERIOD_US;            // 이보다 늦으면 FAULT
const uint32_t CMD_TIMEOUT_MS = 300;               // 보드 측 통신 타임아웃
const uint32_t TORQUE_OFF_AFTER_MS = 5000;         // 타임아웃 후 이 시간 더 명령이 없으면 토크 OFF
const uint8_t BUS_WATCHDOG_20MS = 10;              // 모터 Bus Watchdog 200 ms
const int32_t PROFILE_ACCEL_RAW = 30;              // 214.577 rev/min² 단위 → 약 640°/s²

constexpr float DEG_PER_TICK = 360.0f / 4096.0f;
constexpr float DPS_PER_RAW = 0.229f * 360.0f / 60.0f;  // 1.374 °/s
const uint16_t ADDR_PRESENT_VELOCITY = 128;        // 128~131 속도, 132~135 위치

enum State { OFF, HOLD, TRACK, HOMING, FAULT };
const char *STATE_NAMES[] = {"OFF", "HOLD", "TRACK", "HOMING", "FAULT"};
State state = OFF;

int32_t base_ticks[N];         // 가장 가까운 IDLE 등가 위치(전원 투입 시 0~4095로 초기화되므로 보정)
float pos_deg[N], vel_dps[N];  // IDLE 기준 관절각, 측정 속도
float cmd_dps[N] = {0, 0};     // Pi 명령
uint32_t last_cmd_ms = 0, last_us = 0, home_start_ms = 0;
uint8_t status_count = 0;
bool timeout_reported = false;

int32_t wrapTicks(int32_t t) {  // (−2048, 2048]
  t %= 4096;
  if (t > 2048) t -= 4096;
  if (t <= -2048) t += 4096;
  return t;
}

void sendError(int code, const char *text) {
  Serial.print("E "); Serial.print(code); Serial.print(' '); Serial.println(text);
}

void fault(const char *reason) {
  for (uint8_t i = 0; i < N; ++i) {
    dxl.setGoalVelocity(IDS[i], 0, UNIT_RAW);
    dxl.torqueOff(IDS[i]);
  }
  state = FAULT;
  sendError(4, reason);
}

bool dxlOk() { return dxl.getLastLibErrCode() == DXL_LIB_OK && dxl.getLastStatusPacketError() == 0; }

// 두 모터의 속도·위치를 한 패킷씩(8바이트) 읽는다.
bool readState() {
  for (uint8_t i = 0; i < N; ++i) {
    uint8_t buf[8];
    if (dxl.read(IDS[i], ADDR_PRESENT_VELOCITY, 8, buf, sizeof(buf), 5) != 8 || !dxlOk()) {
      fault("DXL read failed");
      return false;
    }
    int32_t v, p;
    memcpy(&v, buf, 4);
    memcpy(&p, buf + 4, 4);
    pos_deg[i] = (p - base_ticks[i]) * DEG_PER_TICK;
    vel_dps[i] = v * DPS_PER_RAW;
  }
  return true;
}

bool writeVelocity(uint8_t i, float dps) {
  const int32_t raw = lroundf(dps / DPS_PER_RAW);
  if (!dxl.setGoalVelocity(IDS[i], raw, UNIT_RAW) || !dxlOk()) {
    fault("DXL velocity write failed");
    return false;
  }
  return true;
}

bool torqueOffAll() {
  for (uint8_t i = 0; i < N; ++i) {
    if (!dxl.setGoalVelocity(IDS[i], 0, UNIT_RAW) || !dxl.torqueOff(IDS[i])) {
      fault("torque off");
      return false;
    }
  }
  state = OFF;
  return true;
}

// OFF에서 처음 명령을 받으면 토크를 켠다. IDLE에서 너무 멀면 거부한다.
bool enable() {
  if (state == FAULT) { sendError(4, "FAULT: RESET required"); return false; }
  if (state != OFF) return true;
  if (!readState()) return false;
  for (uint8_t i = 0; i < N; ++i) {
    if (fabsf(pos_deg[i]) > ENABLE_LIMIT_DEG[i]) {
      sendError(3, i == 0 ? "pan too far from IDLE; move it by hand" : "tilt too far from IDLE; move it by hand");
      return false;
    }
  }
  for (uint8_t i = 0; i < N; ++i) {
    // watchdog 오류를 지우고, 남은 속도 명령을 0으로 만든 뒤 토크를 켠다.
    if (!dxl.writeControlTableItem(BUS_WATCHDOG, IDS[i], 0) || !dxlOk() ||
        !dxl.writeControlTableItem(PROFILE_ACCELERATION, IDS[i], PROFILE_ACCEL_RAW) || !dxlOk() ||
        !dxl.setGoalVelocity(IDS[i], 0, UNIT_RAW) || !dxlOk() ||
        !dxl.torqueOn(IDS[i]) || !dxlOk() ||
        !dxl.writeControlTableItem(BUS_WATCHDOG, IDS[i], BUS_WATCHDOG_20MS) || !dxlOk()) {
      fault("torque on / watchdog setup");
      return false;
    }
  }
  state = HOLD;
  last_us = micros();
  return true;
}

bool parseVelocity(const char *line, float &pan, float &tilt) {
  char *end;
  pan = strtof(line + 1, &end);
  if (end == line + 1 || !isfinite(pan)) return false;
  const char *cursor = end;
  tilt = strtof(cursor, &end);
  if (end == cursor || !isfinite(tilt)) return false;
  while (isspace(static_cast<unsigned char>(*end))) ++end;
  return *end == '\0';
}

void handleLine(const char *line) {
  const char c = line[0];
  const bool bare = line[1] == '\0';
  float pan, tilt;
  if (c == 'V' && parseVelocity(line, pan, tilt)) {
    last_cmd_ms = millis();
    timeout_reported = false;
    cmd_dps[0] = pan; cmd_dps[1] = tilt;
    // 0 명령은 OFF·HOLD·HOMING에서 생존 신호로만 쓴다(토크를 켜거나 IDLE 복귀를 끊지 않음).
    if (pan == 0 && tilt == 0 && state != TRACK) return;
    if (!enable()) return;
    state = TRACK;
  } else if (c == 'I' && bare) {
    last_cmd_ms = millis();
    timeout_reported = false;
    if (!enable()) return;
    cmd_dps[0] = cmd_dps[1] = 0;
    state = HOMING;
    home_start_ms = millis();
  } else if (c == 'X' && bare) {
    last_cmd_ms = millis();
    cmd_dps[0] = cmd_dps[1] = 0;
    if (state == TRACK || state == HOMING) state = HOLD;
  } else if (c == 'O' && bare) {
    cmd_dps[0] = cmd_dps[1] = 0;
    if (state != OFF && state != FAULT) torqueOffAll();
  } else {
    sendError(2, "bad command");
  }
}

void readCommands() {
  static char line[48];
  static uint8_t used = 0;
  static bool overflow = false;
  for (uint8_t n = 0; n < 64 && Serial.available(); ++n) {
    const char c = Serial.read();
    if (c == '\r' || c == '\n') {
      line[used] = '\0';
      if (overflow) sendError(2, "command too long");
      else if (used) handleLine(line);
      used = 0; overflow = false;
    } else if (!overflow) {
      if (used < sizeof(line) - 1) line[used++] = c;
      else overflow = true;
    }
  }
}

// 한계에 가까워질수록 바깥 방향 속도를 줄이고, 한계 밖에서는 안쪽 방향만 허용한다.
float applyLimits(uint8_t i, float dps) {
  dps = constrain(dps, -SPEED_LIMIT_DPS, SPEED_LIMIT_DPS);
  const float upper = fmaxf(0.0f, LIMIT_GAIN * (LIMIT_DEG[i] - pos_deg[i]));
  const float lower = fminf(0.0f, LIMIT_GAIN * (-LIMIT_DEG[i] - pos_deg[i]));
  return constrain(dps, lower, upper);
}

void sendStatus() {
  Serial.print("S "); Serial.print(millis());
  for (uint8_t i = 0; i < N; ++i) { Serial.print(' '); Serial.print(pos_deg[i], 2); }
  for (uint8_t i = 0; i < N; ++i) { Serial.print(' '); Serial.print(vel_dps[i], 2); }
  Serial.print(' '); Serial.println(STATE_NAMES[state]);
}

void setup() {
  Serial.begin(115200);
  dxl.begin(DXL_BAUD);  // 라이브러리가 OpenCR의 DXL 전원도 켠다.
  dxl.setPortProtocolVersion(2.0);
  delay(500);
  for (uint8_t i = 0; i < N; ++i) {
    if (!dxl.ping(IDS[i])) { fault("ping: check ID/baud/power"); return; }
    if (dxl.getModelNumber(IDS[i]) != XM430_W350) { fault("requires XM430-W350"); return; }
    dxl.torqueOff(IDS[i]);
    const int32_t mode = dxl.readControlTableItem((uint8_t)OPERATING_MODE, IDS[i], 10);
    if (!dxlOk()) { fault("read operating mode"); return; }
    if (mode != OP_VELOCITY && !dxl.setOperatingMode(IDS[i], OP_VELOCITY)) { fault("set velocity mode"); return; }
    const int32_t drive = dxl.readControlTableItem((uint8_t)DRIVE_MODE, IDS[i], 10);
    if (!dxlOk() || (drive & 1)) { fault("Drive Mode reverse bit must be 0"); return; }
    // 속도 모드에서 위치는 전원 투입 때만 0~4095로 초기화되므로, 여기서 한 번 IDLE 기준을 잡는다.
    const int32_t p = dxl.readControlTableItem((uint8_t)PRESENT_POSITION, IDS[i], 10);
    if (!dxlOk()) { fault("read position"); return; }
    base_ticks[i] = p - wrapTicks(p - IDLE_TICKS[i]);
  }
  Serial.println("READY opencr_tracker: V <pan_dps> <tilt_dps> | I | X | O, newline. Status S 50 Hz.");
  last_us = micros();
}

void loop() {
  readCommands();
  const uint32_t now_us = micros();
  const uint32_t dt_us = now_us - last_us;  // unsigned 차분: micros() wrap 대응
  if (dt_us < PERIOD_US) return;
  last_us = now_us;
  if (state == FAULT) return;

  // 1. 상태 읽기 (토크 OFF여도 /joint_states용으로 계속 읽는다)
  if (!readState()) return;

  if (state == HOLD || state == TRACK || state == HOMING) {
    if (dt_us > LATE_US) { fault("control loop late"); return; }
    // 2. 보드 측 통신 타임아웃
    const uint32_t silent_ms = millis() - last_cmd_ms;
    if (silent_ms > CMD_TIMEOUT_MS && state != HOLD) {
      state = HOLD;
      if (!timeout_reported) { sendError(1, "command timeout; stop"); timeout_reported = true; }
    }
    if (silent_ms > CMD_TIMEOUT_MS + TORQUE_OFF_AFTER_MS) {
      if (torqueOffAll()) sendError(1, "command timeout; torque off");
      return;
    }
    // 3. 명령 선택
    float target[N] = {0, 0};
    if (state == TRACK) {
      target[0] = cmd_dps[0]; target[1] = cmd_dps[1];
    } else if (state == HOMING) {
      bool done = true;
      for (uint8_t i = 0; i < N; ++i) {
        if (fabsf(pos_deg[i]) > HOME_TOL_DEG) done = false;
        target[i] = constrain(-HOME_KP * pos_deg[i], -HOME_SPEED_DPS, HOME_SPEED_DPS);
      }
      if (done) { state = HOLD; target[0] = target[1] = 0; }
      else if (millis() - home_start_ms > HOME_TIMEOUT_MS) {
        state = HOLD; target[0] = target[1] = 0;
        sendError(5, "IDLE not reached in time; stop");
      }
    }
    // 4. 제한 후 쓰기
    for (uint8_t i = 0; i < N; ++i) if (!writeVelocity(i, applyLimits(i, target[i]))) return;
  }

  if (++status_count >= STATUS_DIV) { status_count = 0; sendStatus(); }
}
