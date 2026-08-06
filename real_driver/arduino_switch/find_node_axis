#include <SPI.h>
#include <mcp_can.h>

// =========================================================================
// ⚙️ 하드웨어 핀 및 CAN 설정
// =========================================================================
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

const float DT = 0.02f; // 50Hz (20ms 주기)

// =========================================================================
// ⚙️ Welcon 모터 드라이버 3개 (Node 1, 2, 3) × 2개 축 (Axis 1, 2) = 총 6개 후보
// =========================================================================
struct CandidateAxis {
  char key;
  const char* label;
  uint8_t nodeId;
  uint8_t axis;
  int32_t currentCount;
  uint16_t statusWord;
  bool isOnline;
  float testVoltage;
  int assignedJoint; // -1: 미지정, 0: Joint1, 1: Joint2, 2: Joint3, 3: Joint4
};

CandidateAxis candidates[6] = {
  {'a', "Candidate A (Node 1, Axis 1)", 1, 1, 0, 0, false, 0.0f, -1},
  {'b', "Candidate B (Node 1, Axis 2)", 1, 2, 0, 0, false, 0.0f, -1},
  {'c', "Candidate C (Node 2, Axis 1)", 2, 1, 0, 0, false, 0.0f, -1},
  {'d', "Candidate D (Node 2, Axis 2)", 2, 2, 0, 0, false, 0.0f, -1},
  {'e', "Candidate E (Node 3, Axis 1)", 3, 1, 0, 0, false, 0.0f, -1},
  {'f', "Candidate F (Node 3, Axis 2)", 3, 2, 0, 0, false, 0.0f, -1}
};

int selectedCandidateIdx = 0; // 현재 테스트 조작 중인 후보 인덱스 (기본 Candidate A)
unsigned long lastLoopTime = 0;
unsigned long lastLogTime = 0;
uint8_t statPollCounter = 0;
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

void enableCandidateAxis(uint8_t nodeId, uint8_t axis) {
  uint16_t modeObj = (axis == 1) ? 0x6060 : 0x6860;
  uint16_t ctrlObj = (axis == 1) ? 0x6040 : 0x6840;

  sendSdoWriteI8(nodeId, modeObj, 0x00, -11);
  delayMicroseconds(200);

  uint16_t sequence[] = {0x0080, 0x0006, 0x0007, 0x000F};
  for (int s = 0; s < 4; s++) {
    sendSdoWriteI16(nodeId, ctrlObj, 0x00, sequence[s]);
    delayMicroseconds(200);
  }
}

void initHardware() {
  Serial.println(F("▶ CAN 버스 상의 모든 후보 (Node 1~3, Axis 1~2) 드라이버 활성화..."));
  sendNmtStart(0);
  delay(100);

  for (int i = 0; i < 6; i++) {
    enableCandidateAxis(candidates[i].nodeId, candidates[i].axis);
    candidates[i].testVoltage = 0.0f;
    delay(20);
  }
  Serial.println(F("✅ 노드/축 탐색 준비 완료!\n"));
}

void resetAndReinitCanBus() {
  Serial.println(F("\n⚠️ [CAN 통신 경고] MCP2515 CAN 칩 자동 재개통 중..."));
  CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
  CAN.setMode(MCP_NORMAL);
  delay(10);
  initHardware();
  consecutiveCanFailures = 0;
}

// 6개 모든 후보축 피드백 수신 및 응답 여부 판단
void pollCandidates() {
  statPollCounter++;
  bool pollStatus = (statPollCounter % 10 == 0);
  bool anyReadSuccess = false;

  for (int i = 0; i < 6; i++) {
    uint8_t nodeId = candidates[i].nodeId;
    uint16_t posObj = (candidates[i].axis == 1) ? 0x6064 : 0x6864;
    uint16_t statObj = (candidates[i].axis == 1) ? 0x6041 : 0x6841;

    bool posOk = false;
    int32_t posVal = readSdoInt32(nodeId, posObj, 0x00, posOk);
    if (posOk) {
      anyReadSuccess = true;
      candidates[i].currentCount = posVal;
      candidates[i].isOnline = true;
    }

    if (pollStatus) {
      bool statOk = false;
      int32_t statVal = readSdoInt32(nodeId, statObj, 0x00, statOk);
      if (statOk) {
        anyReadSuccess = true;
        candidates[i].statusWord = (uint16_t)(statVal & 0xFFFF);
        uint16_t status = candidates[i].statusWord;

        if ((status & 0x08) || ((status & 0x0027) != 0x0027)) {
          enableCandidateAxis(nodeId, candidates[i].axis);
        }
      }
    }
  }

  if (anyReadSuccess) {
    consecutiveCanFailures = 0;
  } else {
    consecutiveCanFailures++;
    if (consecutiveCanFailures >= 10) {
      resetAndReinitCanBus();
    }
  }
}

// 후보축 전압 인가
void sendCandidateVoltages() {
  for (int i = 0; i < 6; i++) {
    uint8_t nodeId = candidates[i].nodeId;
    uint16_t voltObj = (candidates[i].axis == 1) ? 0x2103 : 0x2903;
    int32_t voltInt = (int32_t)constrain(round(candidates[i].testVoltage), -9500.0f, 9500.0f);

    sendSdoWriteI32(nodeId, voltObj, 0x00, voltInt);
    delayMicroseconds(100);
  }
}

// 현재 할당 및 엔코더 상태 요약 출력
void printSummaryTable() {
  Serial.println(F("\n==================================================================================="));
  Serial.println(F(" 📋 [새 로봇 핑거 노드/축 매핑 탐색 및 엔코더 현황 표]"));
  Serial.println(F("==================================================================================="));
  for (int i = 0; i < 6; i++) {
    Serial.print(F("  [키 '")); Serial.print(candidates[i].key); Serial.print(F("'] "));
    Serial.print(candidates[i].label);
    Serial.print(F(" ➔ 응답: "));
    if (candidates[i].isOnline) {
      Serial.print(F("✅ ONLINE | 엔코더: "));
      Serial.print(candidates[i].currentCount);
      Serial.print(F(" count"));
    } else {
      Serial.print(F("❌ OFF (연결 안됨)"));
    }

    if (candidates[i].assignedJoint >= 0) {
      Serial.print(F(" ➔ 🎯 [Joint "));
      Serial.print(candidates[i].assignedJoint + 1);
      Serial.print(F("에 할당됨]"));
    }
    Serial.println();
  }
  Serial.println(F("==================================================================================="));
  Serial.println(F("👉 전압(+, -)을 흘려 손가락 움직임을 확인한 뒤, '1'~'4' 키를 입력하여 관절을 할당하세요!\n"));
}

// =========================================================================
// 🏁 setup() 및 loop()
// =========================================================================
void setup() {
  Serial.begin(115200);

  Serial.println(F("==================================================================================="));
  Serial.println(F(" 🔍 새 로봇 핑거 관절별 Node ID 및 Axis 식별/탐색 캘리브레이션 툴"));
  Serial.println(F("==================================================================================="));

  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println(F("✅ MCP2515 CAN 초기화 성공 (1Mbps, 8MHz)!"));
    CAN.setMode(MCP_NORMAL);
  } else {
    Serial.println(F("❌ MCP2515 CAN 초기화 실패! 배선을 확인하세요."));
    while (1);
  }

  initHardware();

  Serial.println(F("▶ 조작 가이드:"));
  Serial.println(F("  - [a], [b], [c], [d], [e], [f] : 테스트할 노드/축 후보 선택"));
  Serial.println(F("    a: Node1 Axis1 | b: Node1 Axis2 | c: Node2 Axis1"));
  Serial.println(F("    d: Node2 Axis2 | e: Node3 Axis1 | f: Node3 Axis2"));
  Serial.println(F("  - [+] / [-]                    : 선택된 후보축에 미세 전압(±600mV) 쏘아 관절 움직임 확인"));
  Serial.println(F("  - [Space] 또는 [0]             : 전압 인가 차단 (0mV 정지)"));
  Serial.println(F("  - [1], [2], [3], [4]           : 선택된 후보축을 Joint 1 ~ Joint 4 에 즉시 할당"));
  Serial.println(F("  - [p] / [P]                    : 현재 탐색/할당된 최종 JointConfig 코드 출력"));
  Serial.println(F("===================================================================================\n"));

  printSummaryTable();
}

void loop() {
  unsigned long now = millis();

  // 50Hz 제어 루프
  if (now - lastLoopTime < 20) return;
  lastLoopTime = now;

  // 1. 피드백 수신
  pollCandidates();

  // 2. 시리얼 키보드 명령어 수신
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd >= 'a' && cmd <= 'f') {
      selectedCandidateIdx = cmd - 'a';
      Serial.print(F("\n👉 [후보 선택] "));
      Serial.print(candidates[selectedCandidateIdx].label);
      Serial.println(F(" 선택됨! (+,- 키로 전압을 쏘아보세요)"));
    } else if (cmd >= 'A' && cmd <= 'F') {
      selectedCandidateIdx = cmd - 'A';
      Serial.print(F("\n👉 [후보 선택] "));
      Serial.print(candidates[selectedCandidateIdx].label);
      Serial.println(F(" 선택됨! (+,- 키로 전압을 쏘아보세요)"));
    } else if (cmd == '+' || cmd == '=') {
      candidates[selectedCandidateIdx].testVoltage += 600.0f;
      Serial.print(F("\n⚡ [전압 +600mV] "));
      Serial.print(candidates[selectedCandidateIdx].label);
      Serial.print(F(" ➔ 현재 전압: "));
      Serial.print((int)candidates[selectedCandidateIdx].testVoltage);
      Serial.print(F("mV | 엔코더: "));
      Serial.print(candidates[selectedCandidateIdx].currentCount);
      Serial.println(F(" count"));
    } else if (cmd == '-' || cmd == '_') {
      candidates[selectedCandidateIdx].testVoltage -= 600.0f;
      Serial.print(F("\n⚡ [전압 -600mV] "));
      Serial.print(candidates[selectedCandidateIdx].label);
      Serial.print(F(" ➔ 현재 전압: "));
      Serial.print((int)candidates[selectedCandidateIdx].testVoltage);
      Serial.print(F("mV | 엔코더: "));
      Serial.print(candidates[selectedCandidateIdx].currentCount);
      Serial.println(F(" count"));
    } else if (cmd == ' ' || cmd == '0') {
      for (int i = 0; i < 6; i++) candidates[i].testVoltage = 0.0f;
      Serial.println(F("\n🛑 [전압 차단] 모든 후보축 인가 전압 0mV 차단 정지"));
    } else if (cmd >= '1' && cmd <= '4') {
      int jIdx = cmd - '1';
      // 기존에 동일 관절에 할당된 후보 해제
      for (int i = 0; i < 6; i++) {
        if (candidates[i].assignedJoint == jIdx) candidates[i].assignedJoint = -1;
      }
      candidates[selectedCandidateIdx].assignedJoint = jIdx;
      Serial.print(F("\n🎯 [할당 완료] "));
      Serial.print(candidates[selectedCandidateIdx].label);
      Serial.print(F(" ➔ ⭐ Joint "));
      Serial.print(jIdx + 1);
      Serial.println(F("로 지정되었습니다!"));
    } else if (cmd == 'p' || cmd == 'P') {
      Serial.println(F("\n==================================================================================="));
      Serial.println(F(" 📋 [최종 생성된 JointConfig joints[4] 소스코드]"));
      Serial.println(F("==================================================================================="));
      Serial.println(F("JointConfig joints[4] = {"));
      for (int j = 0; j < 4; j++) {
        int foundIdx = -1;
        for (int i = 0; i < 6; i++) {
          if (candidates[i].assignedJoint == j) { foundIdx = i; break; }
        }
        if (foundIdx >= 0) {
          Serial.print(F("  {\"j")); Serial.print(j + 1); Serial.print(F("\", "));
          Serial.print(candidates[foundIdx].nodeId); Serial.print(F(", "));
          Serial.print(candidates[foundIdx].axis); Serial.print(F(", "));
          Serial.print(candidates[foundIdx].currentCount); Serial.print(F(".0f, ...}, // Node "));
          Serial.print(candidates[foundIdx].nodeId); Serial.print(F(", Axis "));
          Serial.print(candidates[foundIdx].axis); Serial.print(F(" (엔코더: "));
          Serial.print(candidates[foundIdx].currentCount); Serial.println(F(")"));
        } else {
          Serial.print(F("  {\"j")); Serial.print(j + 1); Serial.println(F("\", 0, 0, 0.0f, ...}, // ⚠️ 미할당됨!"));
        }
      }
      Serial.println(F("};"));
      Serial.println(F("==================================================================================="));
      Serial.println(F("👉 위 C++ 배열 코드를 arduino_switch.ino 의 joints[4] 배열에 복사해 넣으세요!\n"));
    }
  }

  // 3. 테스트 전압 인가
  sendCandidateVoltages();

  // 4. 실시간 상태 로그 출력 (200ms 주기)
  if (now - lastLogTime >= 200) {
    lastLogTime = now;

    Serial.print(F("🔎 테스트 중 ["));
    Serial.print(candidates[selectedCandidateIdx].label);
    Serial.print(F("] Volt: "));
    Serial.print((int)candidates[selectedCandidateIdx].testVoltage);
    Serial.print(F("mV | Enc: "));
    Serial.print(candidates[selectedCandidateIdx].currentCount);
    Serial.println(F(" count"));
  }
}
