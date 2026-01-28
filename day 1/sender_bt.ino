#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// Receiver Bluetooth MAC address
uint8_t receiverMAC[6] = { 
  0xC0, 0xCD, 0xD6, 0xCF, 0x36, 0x92 
};

void setup() {
  Serial.begin(115200);

  // true = master mode
  SerialBT.begin("ESP32_SENDER", true);

  delay(2000);   // very important

  Serial.println("Connecting to receiver...");

  bool connected = SerialBT.connect(receiverMAC);

  if (connected) {
    Serial.println("✅ Bluetooth connected");
  } else {
    Serial.println("❌ Connection failed");
  }
}

void loop() {
  if (SerialBT.connected()) {
    SerialBT.println("Hello from ESP32 sender");
    delay(1000);
  }
}
