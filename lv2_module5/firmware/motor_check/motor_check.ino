// 모터 11(수평)·12(수직)가 실제로 움직이는지만 확인하는 펌웨어.
// 업로드만으로는 움직이지 않는다. USB 시리얼(115200)로 명령을 받을 때만 작게 움직인 뒤 토크를 끈다.
//   p  : 두 모터의 모드·토크·현재 위치 출력
//   1  : ID 11 위치 모드 그대로 +5도 → 시작 위치 → -5도 → 시작 위치, 토크 OFF
//   2  : ID 12 속도 모드 그대로 +약5도 → -약5도 → 정지, 토크 OFF
//   x  : 두 모터 즉시 토크 OFF
// EEPROM(동작 모드 등)은 바꾸지 않는다. XM430-W350(1020)이 아니면 움직이지 않는다.
#include <Dynamixel2Arduino.h>

using namespace ControlTableItem;
Dynamixel2Arduino dxl(Serial3, 84);  // OpenCR DXL 포트 / 방향 제어 핀

const uint32_t DXL_BAUD = 1000000;
const float DXL_PROTOCOL = 2.0;
const uint8_t ID_PAN = 11;   // 수평 회전
const uint8_t ID_TILT = 12;  // 수직 회전
const uint16_t MODEL_XM430_W350 = 1020;
const int32_t STEP_TICKS = 57;          // 4096 tick/회전 → 약 5도
const int32_t PAN_PROFILE_VEL = 20;     // 0.229 rpm 단위 → 약 27도/초
const int32_t JOG_VELOCITY = 10;        // 0.229 rpm 단위 → 약 13.7도/초
const uint32_t JOG_MS = 400;            // 13.7도/초 × 0.4초 ≈ 5.5도

bool checkModel(uint8_t id)
{
  if (!dxl.ping(id)) { Serial.print("ERR no response id="); Serial.println(id); return false; }
  uint16_t model = dxl.getModelNumber(id);
  if (model != MODEL_XM430_W350) { Serial.print("ERR unexpected model id="); Serial.print(id); Serial.print(" model="); Serial.println(model); return false; }
  return true;
}

void printState(uint8_t id)
{
  Serial.print("id="); Serial.print(id);
  Serial.print(" mode="); Serial.print(dxl.readControlTableItem(OPERATING_MODE, id));
  Serial.print(" torque="); Serial.print(dxl.readControlTableItem(TORQUE_ENABLE, id));
  Serial.print(" pos="); Serial.print(dxl.getPresentPosition(id));
  Serial.print(" hw_err="); Serial.println(dxl.readControlTableItem(HARDWARE_ERROR_STATUS, id));
}

// 위치 모드에서 목표에 도달하거나 시간 초과까지 기다리며 위치를 출력한다.
void moveAndWait(uint8_t id, int32_t goal)
{
  dxl.setGoalPosition(id, goal);
  uint32_t t0 = millis();
  while (millis() - t0 < 1500)
  {
    if (Serial.available() && Serial.peek() == 'x') return;
    if (abs(dxl.getPresentPosition(id) - goal) <= 3 && !dxl.readControlTableItem(MOVING, id)) break;
    delay(20);
  }
  Serial.print("  goal="); Serial.print(goal); Serial.print(" pos="); Serial.println(dxl.getPresentPosition(id));
}

void testPan()
{
  if (!checkModel(ID_PAN)) return;
  if (dxl.readControlTableItem(OPERATING_MODE, ID_PAN) != OP_POSITION) { Serial.println("ERR id=11 not position mode, skip"); return; }
  int32_t start = dxl.getPresentPosition(ID_PAN);
  Serial.print("TEST id=11 start="); Serial.println(start);
  dxl.writeControlTableItem(PROFILE_VELOCITY, ID_PAN, PAN_PROFILE_VEL);
  dxl.setGoalPosition(ID_PAN, start);  // 토크 ON 순간 튀지 않도록 현재 위치를 목표로
  dxl.torqueOn(ID_PAN);
  moveAndWait(ID_PAN, start + STEP_TICKS);
  moveAndWait(ID_PAN, start);
  moveAndWait(ID_PAN, start - STEP_TICKS);
  moveAndWait(ID_PAN, start);
  dxl.torqueOff(ID_PAN);
  Serial.println("TEST id=11 done");
  printState(ID_PAN);
}

void jog(int32_t vel)
{
  dxl.writeControlTableItem(GOAL_VELOCITY, ID_TILT, vel);
  uint32_t t0 = millis();
  while (millis() - t0 < JOG_MS)
  {
    if (Serial.available() && Serial.peek() == 'x') break;
    delay(10);
  }
  dxl.writeControlTableItem(GOAL_VELOCITY, ID_TILT, 0);
  delay(300);
  Serial.print("  vel="); Serial.print(vel); Serial.print(" pos="); Serial.println(dxl.getPresentPosition(ID_TILT));
}

void testTilt()
{
  if (!checkModel(ID_TILT)) return;
  if (dxl.readControlTableItem(OPERATING_MODE, ID_TILT) != OP_VELOCITY) { Serial.println("ERR id=12 not velocity mode, skip"); return; }
  Serial.print("TEST id=12 start="); Serial.println(dxl.getPresentPosition(ID_TILT));
  dxl.writeControlTableItem(GOAL_VELOCITY, ID_TILT, 0);
  dxl.torqueOn(ID_TILT);
  jog(+JOG_VELOCITY);
  jog(-JOG_VELOCITY);
  dxl.torqueOff(ID_TILT);
  Serial.println("TEST id=12 done");
  printState(ID_TILT);
}

void stopAll()
{
  dxl.writeControlTableItem(GOAL_VELOCITY, ID_TILT, 0);
  dxl.torqueOff(ID_PAN);
  dxl.torqueOff(ID_TILT);
  Serial.println("STOP torque off");
}

void setup()
{
  Serial.begin(115200);
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);
  stopAll();
}

void loop()
{
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'p') { printState(ID_PAN); printState(ID_TILT); }
  else if (c == '1') testPan();
  else if (c == '2') testTilt();
  else if (c == 'x') stopAll();
}
