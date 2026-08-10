#include <SPI.h>
#include <mcp_can.h>

// =========================================================================
// ⚙️ 하드웨어 핀 및 CAN 설정
// =========================================================================
const int SPI_CS_PIN = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN(SPI_CS_PIN); // CAN 객체 생성

// ⚙️ 4개 관절 노드 및 축 정보 정의 (j1 ~ j4)
struct JointInfo {
  const char* name;
  uint8_t nodeId;
  uint8_t axis;
};

JointInfo joints[4] = {
  {"j1 (Node 3, Axis 1)", 3, 1},
  {"j2 (Node 3, Axis 2)", 3, 2},
  {"j3 (Node 2, Axis 1)", 2, 1},
  {"j4 (Node 1, Axis 1)", 1, 1}
};

int32_t encoderCounts[4] = {0, 0, 0, 0};
bool isOnline[4] = {false, false, false, false};

unsigned long lastLoopTime = 0;
unsigned long lastLogTime = 0;

// =========================================================================
// 🛠️ CiA402 SDO CAN 프레임 전송 함수들
// =========================================================================
void sendNmtStart(uint8_t nodeId = 0) {
  byte data[2] = {0x01, nodeId};
  CAN.sendMsgBuf(0x000, 0, 2, data);
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

  while (micros() - startT < 2000) { // 2.0ms 이내 응답 대기
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

void readAllEncoders() {
  for (int i = 0; i < 4; i++) {
    uint8_t nodeId = joints[i].nodeId;
    uint16_t posObj = (joints[i].axis == 1) ? 0x6064 : 0x6864;

    bool posOk = false;
    int32_t posVal = readSdoInt32(nodeId, posObj, 0x00, posOk);

    if (posOk) {
      encoderCounts[i] = posVal;
      isOnline[i] = true;
    } else {
      isOnline[i] = false;
    }
    delayMicroseconds(300); // SDO 데이터 충돌 방지 미세 간격
  }
}

// =========================================================================
// 🏁 setup() 및 loop()
// =========================================================================
void setup() {
  Serial.begin(115200);

  Serial.println(F("====================================================================="));
  Serial.println(F(" 🎯 Welcon 4개 관절 (j1~j4) 실시간 엔코더 카운트 모니터링 툴"));
  Serial.println(F("====================================================================="));

  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println(F("✅ MCP2515 CAN 초기화 성공 (1Mbps, 8MHz)!"));
    CAN.setMode(MCP_NORMAL);
  } else {
    Serial.println(F("❌ MCP2515 CAN 초기화 실패! 배선 및 CS 핀을 확인하세요."));
    while (1);
  }

  sendNmtStart(0);
  delay(100);

  Serial.println(F("▶ 실시간 엔코더 출력 모니터링을 시작합니다...\n"));
}

void loop() {
  unsigned long now = millis();

  // 1초 주기 NMT Operational 유지
  static unsigned long lastNmtTime = 0;
  if (now - lastNmtTime >= 1000) {
    lastNmtTime = now;
    sendNmtStart(0);
  }

  // 50Hz (20ms) 주기로 엔코더 값 읽기
  if (now - lastLoopTime >= 20) {
    lastLoopTime = now;
    readAllEncoders();
  }

  // 100ms 간격으로 시리얼 모니터에 보기 좋게 출력
  if (now - lastLogTime >= 100) {
    lastLogTime = now;

    Serial.print(F("📍 [ENCODER] "));
    for (int i = 0; i < 4; i++) {
      Serial.print(F("J"));
      Serial.print(i + 1);
      Serial.print(F(": "));
      
      if (isOnline[i]) {
        Serial.print(encoderCounts[i]);
        Serial.print(F(" count"));
      } else {
        Serial.print(F("OFFLINE"));
      }

      if (i < 3) Serial.print(F("  |  "));
    }
    Serial.println();
  }
}
