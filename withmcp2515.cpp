#include <SPI.h>
#include "mcp_can.h"

// ESP32 <-> MCP2515 Wiring
#define SPI_CS_PIN     5    // Chip Select (CS)
#define CAN_INT_PIN    2    // Interrupt pin (not used here, but good for receive)

MCP_CAN CAN(SPI_CS_PIN);    // Create CAN controller object

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("ESP32 CAN Transmit Test - NORMAL Mode");

  // Initialize MCP2515 at 500kbps with 8 MHz crystal
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("ERROR: MCP2515 Initialization Failed. Check wiring.");
    while (1); // halt
  }

  // Set CAN mode to NORMAL for real transmission
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN Mode set to NORMAL");
  
  delay(1000);
}

void loop() {
  // CAN frame definition
  unsigned long can_id = 0x100;           // Standard CAN ID
  byte data[] = {0xAB, 0xCD};             // Payload
  byte len = sizeof(data);               // DLC = 2

  // Transmit the CAN frame
  byte sendStatus = CAN.sendMsgBuf(can_id, 0, len, data);

  if (sendStatus == CAN_OK) {
    Serial.print("CAN TX → ID: 0x");
    Serial.print(can_id, HEX);
    Serial.print(" | DLC: ");
    Serial.print(len);
    Serial.print(" | Data: ");
    for (byte i = 0; i < len; i++) {
      Serial.print("0x");
      if (data[i] < 0x10) Serial.print("0"); // pad with 0
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
    Serial.println("✓");
  } else {
    Serial.println("⚠️  Error sending CAN message");
  }

  delay(1000);  // Send one message every second
}
