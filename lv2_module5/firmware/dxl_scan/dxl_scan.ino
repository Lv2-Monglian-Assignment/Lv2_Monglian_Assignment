// OpenCR Serial3(DXL 포트)에 연결된 다이나믹셀의 ID·통신속도·프로토콜·모델을 찾는다.
// 토크를 켜지 않으므로 모터는 움직이지 않는다. USB 시리얼(115200)로 결과 출력, 's' 입력 시 재스캔.
#include <Dynamixel2Arduino.h>

using namespace ControlTableItem;
Dynamixel2Arduino dxl(Serial3, 84);  // OpenCR DXL 포트 / 방향 제어 핀

const uint32_t BAUDS[] = {57600, 1000000, 115200, 2000000, 3000000, 4000000, 9600};
const float PROTOCOLS[] = {2.0, 1.0};

void printItem(const char *name, uint8_t id, uint8_t item)
{
  Serial.print("    "); Serial.print(name); Serial.print("=");
  int32_t v = dxl.readControlTableItem(item, id);
  if (dxl.getLastLibErrCode() == 0) Serial.println(v);
  else Serial.println("n/a");
}

void scanAll()
{
  uint8_t found_total = 0;
  Serial.println("SCAN_START");
  for (uint32_t baud : BAUDS)
  {
    dxl.begin(baud);
    for (float proto : PROTOCOLS)
    {
      dxl.setPortProtocolVersion(proto);
      uint8_t found = 0;
      for (int id = 0; id < 253; id++)
      {
        if (!dxl.ping(id)) continue;
        found++;
        found_total++;
        Serial.print("  FOUND baud="); Serial.print(baud);
        Serial.print(" proto="); Serial.print(proto, 1);
        Serial.print(" id="); Serial.print(id);
        Serial.print(" model_no="); Serial.println(dxl.getModelNumber(id));
        printItem("Firmware_Version", id, FIRMWARE_VERSION);
        printItem("Operating_Mode", id, OPERATING_MODE);
        printItem("Torque_Enable", id, TORQUE_ENABLE);
        printItem("Present_Position", id, PRESENT_POSITION);
        printItem("Min_Position_Limit", id, MIN_POSITION_LIMIT);
        printItem("Max_Position_Limit", id, MAX_POSITION_LIMIT);
        printItem("Velocity_Limit", id, VELOCITY_LIMIT);
        printItem("Present_Input_Voltage", id, PRESENT_INPUT_VOLTAGE);
      }
      Serial.print("baud="); Serial.print(baud);
      Serial.print(" proto="); Serial.print(proto, 1);
      Serial.print(" found="); Serial.println(found);
    }
  }
  Serial.print("SCAN_DONE total="); Serial.println(found_total);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  scanAll();
}

void loop()
{
  if (Serial.available() && Serial.read() == 's') scanAll();
}
