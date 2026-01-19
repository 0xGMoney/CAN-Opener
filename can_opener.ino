extern "C" {
  #include <can_opener.h>
  #include <string.h>
}

#define DEBUG 0

CAN_FRAME frame = {
  .frame_ident = 0,
  .frame_ex_ident = 0,
  .frame_dlc = 0
};

void setup() {
  uint8_t count = 0;

  Serial.begin(9600);
  CAN_OPENER_INIT(CAN_250KBPS);

  if (!SET_MCP_MODE(NORMAL_MODE)) {
    Serial.println(F("MCP is NOT in Normal Mode! Returning..."));
    return;
  } else {
    Serial.println(F("MCP is in Normal Mode..."));
  }

  Serial.println(F("Waiting to receive CAN frame..."));
#if DEBUG
  count = 0;
  while (count != 5) {
    Serial.print(F("Result of MCP_RX_STATUS: 0x"));
    Serial.println(MCP_RX_STATUS(), HEX);
    delay(2500);
    count++;
  }
  return;
#endif
}

void loop() {
  while (!MCP_RX_STATUS()) {
    ;
  }

  Serial.println(F("Reading CAN frame..."));
  MCP_READ_MSG(&frame);
  Serial.print(F("Frame Identifier: 0x"));
  Serial.println(frame.frame_ident, HEX);
  Serial.print(F("Frame Extended Identifier: 0x"));
  Serial.println(frame.frame_ex_ident, HEX);
  Serial.print(F("Frame DLC: "));
  Serial.println(frame.frame_dlc, DEC);
  Serial.println(F("Frame Data Words:"));
  for (int ii = 0; ii < frame.frame_dlc; ii++) {
    Serial.print(" 0x");
    Serial.print(frame.frame_data[ii], HEX);
  }
  memset(&frame, 0, sizeof(frame));
  Serial.println(F("\n------------------------------"));
}
