#include <SPI.h>
#include <mcp_can.h>

// =========================================================================
// ⚙️ 하드웨어 핀 및 CAN 설정
// =========================================================================
const int SPI_CS_PIN = 10;
const int CAN_INT_PIN = 2;

MCP_CAN CAN(SPI_CS_PIN); // CAN 객체 생성

// 물리 상수 설정
const float PULSES_PER_DEGREE = 11.378f;       // 4096 / 360 (엔코더 1도당 카운트)
const float DT = 0.02f;                        // 50Hz (20ms 주기)

// =========================================================================
// ⚙️ 4축 조인트 구성 구조체 (새 로봇 핑거 모니터링 & 진단용)
// =========================================================================
struct JointConfig {
  const char* name;
  uint8_t nodeId;
  uint8_t axis;
};

// 4개 관절 노드 및 축 할당 매핑 (필요 시 수치 수정 가능)
JointConfig joints[4] = {
  {"Joint 1", 3, 1}, // 조인트 1: Node 3, Axis 1
  {"Joint 2", 2, 1}, // 조인트 2: Node 2, Axis 1
  {"Joint 3", 1, 1}, // 조인트 3: Node 1, Axis 1
  {"Joint 4", 3, 2}  // 조인트 4: Node 3, Axis 2
};

struct JointState {
  int32_t currentCount;
  uint16_t statusWord;
  bool isConnected;
  float testVoltage; // 진단용 미세 전압 (mV)
};

JointState jointStates[4];

unsigned long lastLoopTime = 0;
unsigned long lastLogTime = 0;
uint8_t statPollCounter = 0;
int selectedTestJoint = -1; // -1: 선택 없음, 0~3: 테스트 중인 조인트
uint8_t consecutiveCanFailures = 0;

// =========================================================================
// 🛠️ CiA402 SDO CAN 프레임 전송 함수들
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

// SDO 동기식 정밀 읽기 (타임아웃 1.5ms)
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

// 드라이버 Operation Enable 가동 함수
void enableNodeAxis(uint8_t nodeId, uint8_t axis) {
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
  Serial.println(F("▶ Welcon 모터 드라이버 (j1~j4) 초기화 및 Operation Enable 진행 중..."));
  sendNmtStart(0);
  delay(100);

  for (int i = 0; i < 4; i++) {
    enableNodeAxis(joints[i].nodeId, joints[i].axis);
    jointStates[i].testVoltage = 0.0f;
    delay(20);
  }
  Serial.println(F("✅ 하드웨어 모니터링 가동 준비 완료!\n"));
}

void resetAndReinitCanBus() {
  Serial.println(F("\n⚠️ [CAN 통신 경고] 통신 무응답 감지 -> MCP2515 CAN 칩 자동 재개통 중..."));
  CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
  CAN.setMode(MCP_NORMAL);
  delay(10);
  initHardware();
  consecutiveCanFailures = 0;
}

// SDO 피드백 수신 (위치 및 상태 워드 매 루프 읽기)
void readJointFeedbacks() {
  statPollCounter++;
  bool pollStatus = (statPollCounter % 10 == 0); // 200ms 마다 Statusword 폴링
  bool anyReadSuccess = false;

  for (int i = 0; i < 4; i++) {
    uint8_t nodeId = joints[i].nodeId;
    uint16_t posObj = (joints[i].axis == 1) ? 0x6064 : 0x6864;
    uint16_t statObj = (joints[i].axis == 1) ? 0x6041 : 0x6841;

    bool posOk = false;
    int32_t posVal = readSdoInt32(nodeId, posObj, 0x00, posOk);
    if (posOk) {
      anyReadSuccess = true;
      jointStates[i].currentCount = posVal;
      jointStates[i].isConnected = true;
    }

    if (pollStatus) {
      bool statOk = false;
      int32_t statVal = readSdoInt32(nodeId, statObj, 0x00, statOk);
      if (statOk) {
        anyReadSuccess = true;
        jointStates[i].statusWord = (uint16_t)(statVal & 0xFFFF);
        uint16_t status = jointStates[i].statusWord;

        if ((status & 0x08) || ((status & 0x0027) != 0x0027)) {
          enableNodeAxis(joints[i].nodeId, joints[i].axis);
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

// 각 관절에 전압(mV) SDO 전송
void sendVoltages(float voltages[4]) {
  for (int i = 0; i < 4; i++) {
    uint8_t nodeId = joints[i].nodeId;
    uint16_t voltObj = (joints[i].axis == 1) ? 0x2103 : 0x2903;
    int32_t voltInt = (int32_t)constrain(round(voltages[i]), -9500.0f, 9500.0f);

    sendSdoWriteI32(nodeId, voltObj, 0x00, voltInt);
    delayMicroseconds(100);
  }
}

// =========================================================================
// 🏁 setup() 및 loop()
// =========================================================================
void setup() {
  Serial.begin(115200);

  Serial.println(F("==================================================================================="));
  Serial.println(F(" 🎯 새 로봇 핑거 관절별 Node/Axis 할당 정보 및 절대 엔코더 실시간 모니터"));
  Serial.println(F("==================================================================================="));

  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println(F("✅ MCP2515 CAN 초기화 성공 (1Mbps, 8MHz)!"));
    CAN.setMode(MCP_NORMAL);
  } else {
    Serial.println(F("❌ MCP2515 CAN 초기화 실패! 배선 및 전원을 확인하세요."));
    while (1);
  }

  initHardware();

  Serial.println(F("▶ 키보드 모니터링 & 진단 조작 안내:"));
  Serial.println(F("  [1], [2], [3], [4] : 진단 테스트할 조인트 선택 (1~4)"));
  Serial.println(F("  [+] / [-]        : 선택한 조인트에 미세 전압 (±500mV) 흘려 구동 확인"));
  Serial.println(F("  [Space] 또는 [0] : 전압 인가 차단 (0mV 정지)"));
  Serial.println(F("  [c] / [C]        : 현재 4개 관절의 절대 엔코더 수치(alignCount) 캡처 요약 출력"));
  Serial.println(F("===================================================================================\n"));
}

void loop() {
  unsigned long now = millis();

  // 50Hz 제어 루프
  if (now - lastLoopTime < 20) return;
  lastLoopTime = now;

  // 1. 엔코더 피드백 수신
  readJointFeedbacks();

  // 2. 키보드 명령 수신 및 처리
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    if (cmd == '1') {
      selectedTestJoint = 0;
      Serial.println(F("\n👉 [테스트 선택] Joint 1 (Node 3, Axis 1) 선택됨"));
    } else if (cmd == '2') {
      selectedTestJoint = 1;
      Serial.println(F("\n👉 [테스트 선택] Joint 2 (Node 2, Axis 1) 선택됨"));
    } else if (cmd == '3') {
      selectedTestJoint = 2;
      Serial.println(F("\n👉 [테스트 선택] Joint 3 (Node 1, Axis 1) 선택됨"));
    } else if (cmd == '4') {
      selectedTestJoint = 3;
      Serial.println(F("\n👉 [테스트 선택] Joint 4 (Node 3, Axis 2) 선택됨"));
    } else if (cmd == '+' || cmd == '=') {
      if (selectedTestJoint >= 0 && selectedTestJoint < 4) {
        jointStates[selectedTestJoint].testVoltage += 500.0f;
        Serial.print(F("\n⚡ [전압 +500mV 인가] "));
        Serial.print(joints[selectedTestJoint].name);
        Serial.print(F(" ➔ 현재 전압: "));
        Serial.print((int)jointStates[selectedTestJoint].testVoltage);
        Serial.println(F(" mV"));
      } else {
        Serial.println(F("\n⚠️ 먼저 숫자 키 [1], [2], [3], [4]를 눌러 조인트를 선택하세요!"));
      }
    } else if (cmd == '-' || cmd == '_') {
      if (selectedTestJoint >= 0 && selectedTestJoint < 4) {
        jointStates[selectedTestJoint].testVoltage -= 500.0f;
        Serial.print(F("\n⚡ [전압 -500mV 인가] "));
        Serial.print(joints[selectedTestJoint].name);
        Serial.print(F(" ➔ 현재 전압: "));
        Serial.print((int)jointStates[selectedTestJoint].testVoltage);
        Serial.println(F(" mV"));
      } else {
        Serial.println(F("\n⚠️ 먼저 숫자 키 [1], [2], [3], [4]를 눌러 조인트를 선택하세요!"));
      }
    } else if (cmd == ' ' || cmd == '0') {
      for (int i = 0; i < 4; i++) jointStates[i].testVoltage = 0.0f;
      selectedTestJoint = -1;
      Serial.println(F("\n🛑 [전압 차단] 모든 조인트 인가 전압 0mV 차단 및 선택 해제"));
    } else if (cmd == 'c' || cmd == 'C') {
      Serial.println(F("\n========================================================="));
      Serial.println(F(" 📋 [현재 로봇 핑거 정렬 엔코더 카운트 데이터 캡처]"));
      Serial.println(F("========================================================="));
      for (int i = 0; i < 4; i++) {
        Serial.print(F("  "));
        Serial.print(joints[i].name);
        Serial.print(F(" (Node ")); Serial.print(joints[i].nodeId);
        Serial.print(F(", Axis ")); Serial.print(joints[i].axis);
        Serial.print(F(") ➔ alignCount = "));
        Serial.print(jointStates[i].currentCount);
        Serial.println(F(".0f;"));
      }
      Serial.println(F("========================================================="));
      Serial.println(F("👉 위 수치를 아두이노 코드의 JointConfig alignCount에 반영하세요!\n"));
    }
  }

  // 3. 테스트 전압 인가
  float currentVoltages[4];
  for (int i = 0; i < 4; i++) {
    currentVoltages[i] = jointStates[i].testVoltage;
  }
  sendVoltages(currentVoltages);

  // 4. 실시간 노드/축 할당 상태 및 절대 엔코더 수치 출력 (200ms = 5Hz 주기)
  if (now - lastLogTime >= 200) {
    lastLogTime = now;

    Serial.print(F("📍 [J1: N")); Serial.print(joints[0].nodeId); Serial.print(F("-A")); Serial.print(joints[0].axis);
    Serial.print(F("] Raw: ")); Serial.print(jointStates[0].currentCount);

    Serial.print(F(" | [J2: N")); Serial.print(joints[1].nodeId); Serial.print(F("-A")); Serial.print(joints[1].axis);
    Serial.print(F("] Raw: ")); Serial.print(jointStates[1].currentCount);

    Serial.print(F(" | [J3: N")); Serial.print(joints[2].nodeId); Serial.print(F("-A")); Serial.print(joints[2].axis);
    Serial.print(F("] Raw: ")); Serial.print(jointStates[2].currentCount);

    Serial.print(F(" | [J4: N")); Serial.print(joints[3].nodeId); Serial.print(F("-A")); Serial.print(joints[3].axis);
    Serial.print(F("] Raw: ")); Serial.print(jointStates[3].currentCount);

    if (selectedTestJoint >= 0) {
      Serial.print(F(" | ⚡[테스트중: J")); Serial.print(selectedTestJoint + 1);
      Serial.print(F(" = ")); Serial.print((int)jointStates[selectedTestJoint].testVoltage); Serial.print(F("mV]"));
    }
    Serial.println();
  }
}
