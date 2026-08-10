#include <SPI.h>
#include <mcp_can.h>

// =========================================================================
// 🎯 Welcon 조인트 1번 (Node 3, Axis 1) 정밀 고장 및 원인 정밀 진단 전용 아두이노 스케치
// =========================================================================
// [조인트 및 노드 매핑]
//  - 조인트 1번 (j1): Node 3, Axis 1 (0x6064 / 0x2103 / 0x6041)
//  - 조인트 2번 (j2): Node 3, Axis 2 (0x6864 / 0x2903 / 0x6841)
// =========================================================================

const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

const uint8_t TARGET_NODE = 3; // 조인트 1번 / 2번 Node ID = 3

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

  while (micros() - startT < 3000) {
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

void decodeCiA402Status(uint16_t status) {
  Serial.print(F(" [0x")); Serial.print(status, HEX); Serial.print(F("] -> "));
  if (status & 0x0008) {
    Serial.println(F("❌ FAULT (드라이버 에러/폴트 상태!)"));
  } else if ((status & 0x006F) == 0x0027) {
    Serial.println(F("✅ Operation Enabled (정상 전력 인가 가능 상태)"));
  } else if ((status & 0x004F) == 0x0023) {
    Serial.println(F("⚠️ Switched ON (전력 준비 상태)"));
  } else if ((status & 0x004F) == 0x0021) {
    Serial.println(F("⚠️ Ready to Switch ON (대기 상태)"));
  } else if ((status & 0x004F) == 0x0040) {
    Serial.println(F("⚠️ Switch ON Disabled (비활성화 상태)"));
  } else {
    Serial.println(F("❓ Unknown/Special State"));
  }
}

void decodeErrorCode(uint16_t errCode) {
  if (errCode == 0x0000) {
    Serial.println(F("  ✅ 에러 없음 (0x0000 No Error)"));
    return;
  }
  Serial.print(F("  ❌ 에러 코드 감지 [0x")); Serial.print(errCode, HEX); Serial.print(F("]: "));
  switch (errCode) {
    case 0x2310: Serial.println(F("과전류 (Continuous Over Current)")); break;
    case 0x3210: Serial.println(F("DC 링크 과전압 (DC Link Over Voltage)")); break;
    case 0x3220: Serial.println(F("DC 링크 저전압 (DC Link Under Voltage)")); break;
    case 0x4210: Serial.println(F("드라이버 과열 (Drive Over Temperature)")); break;
    case 0x7300: Serial.println(F("엔코더 통신/신호 오류 (Sensor Feedback Error)")); break;
    case 0x8611: Serial.println(F("위치 편차 과다 (Following Error Fault)")); break;
    case 0xFF00: Serial.println(F("모터 상선 단선/결상 오류 (Motor Phase Loss/Disconnected)")); break;
    default: Serial.println(F("드라이버 내부 보호 동작 활성화")); break;
  }
}

void diagnoseAxis(uint8_t nodeId, uint8_t axis) {
  Serial.println(F("---------------------------------------------------------"));
  Serial.print(F("🔍 [진단] Node ")); Serial.print(nodeId);
  Serial.print(F(" - Axis ")); Serial.print(axis);
  if (nodeId == 3 && axis == 1) Serial.println(F(" (조인트 1번 - j1)"));
  else if (nodeId == 3 && axis == 2) Serial.println(F(" (조인트 2번 - j2)"));
  else Serial.println();
  Serial.println(F("---------------------------------------------------------"));

  uint16_t posObj  = (axis == 1) ? 0x6064 : 0x6864;
  uint16_t statObj = (axis == 1) ? 0x6041 : 0x6841;
  uint16_t errObj  = (axis == 1) ? 0x603F : 0x683F;
  uint16_t modeObj = (axis == 1) ? 0x6061 : 0x6861;
  uint16_t ctrlObj = (axis == 1) ? 0x6040 : 0x6840;
  uint16_t voltObj = (axis == 1) ? 0x2103 : 0x2903;

  // 1. 엔코더 읽기
  bool okPos = false;
  int32_t initialPos = readSdoInt32(nodeId, posObj, 0x00, okPos);
  if (okPos) {
    Serial.print(F("1. 엔코더 피드백 (0x")); Serial.print(posObj, HEX); Serial.print(F("): "));
    Serial.print(initialPos); Serial.println(F(" count (통신 정상)"));
  } else {
    Serial.print(F("1. 엔코더 피드백 (0x")); Serial.print(posObj, HEX); Serial.println(F("): ❌ 통신 응답 없음!"));
    return;
  }

  // 2. StatusWord 읽기
  bool okStat = false;
  int32_t statVal = readSdoInt32(nodeId, statObj, 0x00, okStat);
  if (okStat) {
    Serial.print(F("2. CiA402 상태 (0x")); Serial.print(statObj, HEX); Serial.print(F("):"));
    decodeCiA402Status((uint16_t)(statVal & 0xFFFF));
  }

  // 3. ErrorCode 읽기
  bool okErr = false;
  int32_t errVal = readSdoInt32(nodeId, errObj, 0x00, okErr);
  if (okErr) {
    decodeErrorCode((uint16_t)(errVal & 0xFFFF));
  }

  // 4. Mode of Operation Display 읽기
  bool okMode = false;
  int32_t modeVal = readSdoInt32(nodeId, modeObj, 0x00, okMode);
  if (okMode) {
    Serial.print(F("3. 모드 설정 (0x")); Serial.print(modeObj, HEX); Serial.print(F("): "));
    Serial.print((int8_t)modeVal);
    if ((int8_t)modeVal == -11) {
      Serial.println(F(" (정상: Custom Voltage Mode)"));
    } else {
      Serial.println(F(" ❌ 비정상! (-11 전압 제어 모드가 아님)"));
    }
  }

  // 5. CiA402 강제 가동 (Reset -> Shutdown -> SwitchON -> Enable)
  Serial.println(F("\n▶ CiA402 State Machine 리셋 및 Operation Enable 전송..."));
  uint16_t modeCmdObj = (axis == 1) ? 0x6060 : 0x6860;
  sendSdoWriteI8(nodeId, modeCmdObj, 0x00, -11);
  delay(10);
  sendSdoWriteI16(nodeId, ctrlObj, 0x00, 0x0080); delay(10); // Fault Reset
  sendSdoWriteI16(nodeId, ctrlObj, 0x00, 0x0006); delay(10); // Shutdown
  sendSdoWriteI16(nodeId, ctrlObj, 0x00, 0x0007); delay(10); // Switch ON
  sendSdoWriteI16(nodeId, ctrlObj, 0x00, 0x000F); delay(50); // Enable Operation

  // 재확인
  statVal = readSdoInt32(nodeId, statObj, 0x00, okStat);
  Serial.print(F("  가동 후 상태:"));
  decodeCiA402Status((uint16_t)(statVal & 0xFFFF));

  // 6. 전압 주입 테스트 (Pulse Voltage Test +2000mV / -2000mV)
  Serial.println(F("\n⚡ [전압 인가 테스트] +2000mV (2.0V) 1.5초간 출력 시도..."));
  int32_t posStart = readSdoInt32(nodeId, posObj, 0x00, okPos);
  
  for (int i = 0; i < 75; i++) { // 1.5초간 지속 전압 주입
    sendSdoWriteI32(nodeId, voltObj, 0x00, 2000);
    delay(20);
  }
  sendSdoWriteI32(nodeId, voltObj, 0x00, 0); // 전압 차단
  delay(100);

  int32_t posEnd = readSdoInt32(nodeId, posObj, 0x00, okPos);
  int32_t deltaPos = posEnd - posStart;

  Serial.print(F("📊 +2000mV 인가 결과 -> 이동 변위: "));
  Serial.print(deltaPos); Serial.println(F(" count"));

  if (abs(deltaPos) > 15) {
    Serial.println(F("🎉 [결과 SUCCESS] 모터 및 드라이버 회로 정상 작동 확인!"));
  } else {
    Serial.println(F("❌ [결과 FAILURE] 전압을 주었으나 모터 변위가 0에 가깝습니다!"));
    Serial.println(F("  ➔ 원인 1: 모터 3상 케이블(U,V,W) 단선 또는 커넥터 이탈"));
    Serial.println(F("  ➔ 원인 2: 모터 드라이버 FET 출력 단계 퓨즈/하드웨어 손상"));
    Serial.println(F("  ➔ 원인 3: 기계적 구동부 잼(Jamming) 또는 브레이크 잠김"));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println(F("========================================================="));
  Serial.println(F(" 🔬 Welcon Node 3 (조인트 1번: Axis 1) 모터/드라이버 정밀 진단 툴"));
  Serial.println(F("========================================================="));

  if (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println(F("✅ MCP2515 CAN 칩 연결 성공!"));
    CAN.setMode(MCP_NORMAL);
  } else {
    Serial.println(F("❌ MCP2515 CAN 칩 연결 실패!"));
    while (1);
  }

  sendNmtStart(0);
  delay(100);

  // Axis 1 (조인트 1번) 및 Axis 2 (조인트 2번) 정밀 진단 실행
  diagnoseAxis(TARGET_NODE, 1);
  delay(1000);
  diagnoseAxis(TARGET_NODE, 2);
}

void loop() {
  // 1초마다 조인트 1번/2번 실시간 엔코더 및 에러 모니터링
  static unsigned long lastT = 0;
  if (millis() - lastT >= 1000) {
    lastT = millis();

    bool ok1 = false, ok2 = false;
    int32_t pos1 = readSdoInt32(TARGET_NODE, 0x6064, 0x00, ok1);
    int32_t pos2 = readSdoInt32(TARGET_NODE, 0x6864, 0x00, ok2);
    int32_t err1 = readSdoInt32(TARGET_NODE, 0x603F, 0x00, ok1);
    int32_t err2 = readSdoInt32(TARGET_NODE, 0x683F, 0x00, ok2);

    Serial.print(F("📍 Node 3 Realtime -> J1(Axis 1) Pos: "));
    Serial.print(pos1);
    Serial.print(F(" (Err:0x")); Serial.print(err1, HEX);
    Serial.print(F(") | J2(Axis 2) Pos: "));
    Serial.print(pos2);
    Serial.print(F(" (Err:0x")); Serial.print(err2, HEX);
    Serial.println(F(")"));
  }
}
