#include <SPI.h>
#include <mcp_can.h>

// =========================================================================
// ⚙️ 하드웨어 핀 및 CAN 설정
// =========================================================================
const int SPI_CS_PIN = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN(SPI_CS_PIN); // CAN 객체 생성

// 🎯 진단 대상: 조인트 3번 (Node 1, Axis 1)
const uint8_t TARGET_NODE_ID = 1;
const uint8_t TARGET_AXIS = 1;

float testVoltage = 0.0f;          // 실시간 인가 전압 (mV)
int32_t currentEncoderCount = 0;   // 실시간 엔코더 카운트
uint16_t currentStatusWord = 0;    // 실시간 상태워드 (Hex)
bool isDriveOnline = false;

unsigned long lastLoopTime = 0;
unsigned long lastLogTime = 0;
uint8_t consecutiveCanFailures = 0;

// =========================================================================
// 🛠️ SDO CAN 프레임 전송 함수들
// =========================================================================
void sendNmtStart(uint8_t nodeId = 0) {
  byte data[2] = {0x01, nodeId};
  CAN.sendMsgBuf(0x000, 0, 2, data);
}

void sendSdoWriteI8(uint8_t nodeId, uint16_t index, uint8_t subindex, int8_t value) {
  byte data[8] = {
    0x2F, (byte)(index & 0xFF), (byte)((index >> 8) & 0xFF), subindex, (byte)value, 0x00, 0x00, 0x00
  };
  CAN.sendMsgBuf(0x600 + nodeId, 0, 8, data);
}

void sendSdoWriteI16(uint8_t nodeId, uint16_t index, uint8_t subindex, int16_t value) {
  byte data[8] = {
    0x2B, (byte)(index & 0xFF), (byte)((index >> 8) & 0xFF), subindex,
    (byte)(value & 0xFF), (byte)((value >> 8) & 0xFF), 0x00, 0x00
  };
  CAN.sendMsgBuf(0x600 + nodeId, 0, 8, data);
}

void sendSdoWriteI32(uint8_t nodeId, uint16_t index, uint8_t subindex, int32_t value) {
  byte data[8] = {
    0x23, (byte)(index & 0xFF), (byte)((index >> 8) & 0xFF), subindex,
    (byte)(value & 0xFF), (byte)((value >> 8) & 0xFF), (byte)((value >> 16) & 0xFF), (byte)((value >> 24) & 0xFF)
  };
  CAN.sendMsgBuf(0x600 + nodeId, 0, 8, data);
}

void sendSdoRead(uint8_t nodeId, uint16_t index, uint8_t subindex) {
  byte data[8] = {
    0x40, (byte)(index & 0xFF), (byte)((index >> 8) & 0xFF), subindex, 0x00, 0x00, 0x00, 0x00
  };
  CAN.sendMsgBuf(0x600 + nodeId, 0, 8, data);
}

int32_t readSdoInt32(uint8_t nodeId, uint16_t index, uint8_t subindex, bool &success) {
  sendSdoRead(nodeId, index, subindex);
  unsigned long startT = micros();
  success = false;

  while (micros() - startT < 1500) {
    if (CAN.checkReceive() == CAN_MSGAVAIL) {
      unsigned long rxId = 0;
      byte len = 0;
      byte rxBuf[8];

      CAN.readMsgBuf(&rxId, &len, rxBuf);

      if (rxId == (0x580 + nodeId) && len >= 8) {
        uint16_t rxIndex = rxBuf[1] | (rxBuf[2] << 8);
        if (rxIndex == index && rxBuf[3] == subindex) {
          int32_t val = (int32_t)(rxBuf[4] | ((uint32_t)rxBuf[5] << 8) | ((uint32_t)rxBuf[6] << 16) | ((uint32_t)rxBuf[7] << 24));
          success = true;
          return val;
        }
      }
    }
  }
  return 0;
}

// Node 1 Axis 1 드라이버 Operation Enable 가동 함수 (CiA402 10ms 안정성 전이 지연 적용)
void enableJoint3Drive() {
  uint16_t modeObj = (TARGET_AXIS == 1) ? 0x6060 : 0x6860;
  uint16_t ctrlObj = (TARGET_AXIS == 1) ? 0x6040 : 0x6840;

  sendSdoWriteI8(TARGET_NODE_ID, modeObj, 0x00, -11);
  delay(15);

  uint16_t sequence[] = {0x0080, 0x0006, 0x0007, 0x000F};
  for (int s = 0; s < 4; s++) {
    sendSdoWriteI16(TARGET_NODE_ID, ctrlObj, 0x00, sequence[s]);
    delay(15);
  }
}

void initHardware() {
  Serial.println(F("▶ 조인트 3번 (Node 1, Axis 1) 모터 드라이버 가동 준비 중..."));
  sendNmtStart(0);
  delay(100);

  enableJoint3Drive();
  testVoltage = 0.0f;
  delay(50);
  Serial.println(F("✅ 조인트 3번 단독 진단 모드 준비 완료!\n"));
}

void pollJoint3() {
  uint16_t posObj = (TARGET_AXIS == 1) ? 0x6064 : 0x6864;
  uint16_t statObj = (TARGET_AXIS == 1) ? 0x6041 : 0x6841;

  // 1. 위치 읽기
  bool posOk = false;
  int32_t posVal = readSdoInt32(TARGET_NODE_ID, posObj, 0x00, posOk);
  if (posOk) {
    currentEncoderCount = posVal;
    isDriveOnline = true;
  }

  delayMicroseconds(500); // SDO 충돌 방지 미세 대기

  // 2. 상태 워드 읽기 및 Fault 시 자동 리셋
  bool statOk = false;
  int32_t statVal = readSdoInt32(TARGET_NODE_ID, statObj, 0x00, statOk);
  if (statOk) {
    currentStatusWord = (uint16_t)(statVal & 0xFFFF);
  }

  if (posOk || statOk) {
    consecutiveCanFailures = 0;
  } else {
    consecutiveCanFailures++;
    if (consecutiveCanFailures >= 10) {
      CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
      CAN.setMode(MCP_NORMAL);
      delay(10);
      initHardware();
      consecutiveCanFailures = 0;
    }
  }
}

void sendJoint3Voltage(float volt) {
  uint16_t voltObj = (TARGET_AXIS == 1) ? 0x2103 : 0x2903;
  int32_t voltInt = (int32_t)constrain(round(volt), -11000.0f, 11000.0f);

  sendSdoWriteI32(TARGET_NODE_ID, voltObj, 0x00, voltInt);
}

// =========================================================================
// 🏁 setup() 및 loop()
// =========================================================================
void setup() {
  Serial.begin(115200);

  Serial.println(F("==================================================================================="));
  Serial.println(F(" 🔬 Welcon 조인트 3번 (Node 1, Axis 1) 모터 및 드라이버 단독 진단/테스트 툴"));
  Serial.println(F("==================================================================================="));

  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println(F("✅ MCP2515 CAN 통신 개통 성공 (1Mbps, 8MHz)!"));
    CAN.setMode(MCP_NORMAL);
  } else {
    Serial.println(F("❌ MCP2515 CAN 초기화 실패! 배선 및 크리스탈 전압을 확인하세요."));
    while (1);
  }

  initHardware();

  Serial.println(F("▶ 테스트 키보드 명령어:"));
  Serial.println(F("  [1] : 미세 전압  +1.0V (+1000mV) 인가"));
  Serial.println(F("  [2] : 중간 전압  +3.0V (+3000mV) 인가"));
  Serial.println(F("  [3] : 강한 전압  +6.0V (+6000mV) 인가"));
  Serial.println(F("  [4] : 최대 전압  +9.5V (+9500mV) 인가"));
  Serial.println(F("  [+] / [-] : 전압 ±500mV 수동 미세 조절"));
  Serial.println(F("  [0] 또는 [Space] : 인가 전압 0mV (즉시 정지)"));
  Serial.println(F("  [r] : 드라이버 Fault 해제 및 강제 재활성화"));
  Serial.println(F("===================================================================================\n"));
}

void loop() {
  unsigned long now = millis();

  // 50Hz 제어 루프
  if (now - lastLoopTime < 20) return;
  lastLoopTime = now;

  // 1. 조인트 3번 상태 피드백 수신
  pollJoint3();

  // 2. 키보드 명령어 수신 및 전압 변경
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == '1') {
      testVoltage = 1000.0f;
      Serial.println(F("\n⚡ [테스트 전압] +1.0V (+1000mV) 인가 시작"));
    } else if (cmd == '2') {
      testVoltage = 3000.0f;
      Serial.println(F("\n⚡ [테스트 전압] +3.0V (+3000mV) 인가 시작"));
    } else if (cmd == '3') {
      testVoltage = 6000.0f;
      Serial.println(F("\n⚡ [테스트 전압] +6.0V (+6000mV) 인가 시작"));
    } else if (cmd == '4') {
      testVoltage = 9500.0f;
      Serial.println(F("\n⚡ [테스트 전압] +9.5V (+9500mV) 인가 시작"));
    } else if (cmd == '+' || cmd == '=') {
      testVoltage += 500.0f;
      Serial.print(F("\n⚡ [전압 증가] 현재 전압: ")); Serial.print((int)testVoltage); Serial.println(F(" mV"));
    } else if (cmd == '-' || cmd == '_') {
      testVoltage -= 500.0f;
      Serial.print(F("\n⚡ [전압 감소] 현재 전압: ")); Serial.print((int)testVoltage); Serial.println(F(" mV"));
    } else if (cmd == ' ' || cmd == '0') {
      testVoltage = 0.0f;
      Serial.println(F("\n🛑 [전압 차단] 인가 전압 0mV 정지"));
    } else if (cmd == 'r' || cmd == 'R') {
      initHardware();
      Serial.println(F("\n🔄 [드라이버 리셋] Node 1 Axis 1 리셋 및 재가동 완료"));
    }
  }

  // 3. 조인트 3번에 전압 인가
  sendJoint3Voltage(testVoltage);

  // 4. 실시간 상태 로그 출력 (200ms 마다)
  if (now - lastLogTime >= 200) {
    lastLogTime = now;

    Serial.print(F("📍 [Joint 3 진단] Volt: "));
    Serial.print((int)testVoltage);
    Serial.print(F(" mV | Enc: "));
    Serial.print(currentEncoderCount);
    Serial.print(F(" count | Status: 0x"));
    if (currentStatusWord < 0x1000) Serial.print(F("0"));
    if (currentStatusWord < 0x0100) Serial.print(F("0"));
    if (currentStatusWord < 0x0010) Serial.print(F("0"));
    Serial.print(currentStatusWord, HEX);

    if (currentStatusWord & 0x08) {
      Serial.print(F(" ❌ [FAULT 발생!]"));
    } else if ((currentStatusWord & 0x0027) == 0x0027) {
      Serial.print(F(" ✅ [정상 가동중 (Operation Enabled)]"));
    } else {
      Serial.print(F(" ⚠️ [드라이버 대기/비활성화]"));
    }
    Serial.println();
  }
}
