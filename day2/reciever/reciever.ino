#include "BluetoothSerial.h"
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"

BluetoothSerial SerialBT;
RTC_DS3231 rtc;

// SETTINGS
const char* remoteName = "ESP32_IMU_Transmitter";
const int chipSelect = 5;

void setup() {
  // 1. Start Serial with a slight delay to clear boot-up noise
  Serial.begin(115200);
  delay(2000); 
  Serial.println("\n--- Logger Initializing ---");

  // 2. Initialize I2C for RTC
  Wire.begin(21, 22); // Explicitly define SDA, SCL
  if (!rtc.begin()) {
    Serial.println("CRITICAL: RTC not found! Check wiring on GPIO 21/22.");
    while (1); 
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, resetting time...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // 3. Initialize SD Card
  Serial.print("Initializing SD Card... ");
  if (!SD.begin(chipSelect)) {
    Serial.println("FAILED! Check SD Card or Pins.");
    // We don't stop here, so we can at least see Bluetooth data in Serial
  } else {
    Serial.println("SUCCESS.");
  }

  // 4. Start Bluetooth
  Serial.println("Starting Bluetooth Master...");
  SerialBT.begin("ESP32_Logger", true); 
  
  Serial.printf("Searching for: %s...\n", remoteName);
  
  // Try to connect (this can take up to 10 seconds)
  if (SerialBT.connect(remoteName)) {
    Serial.println("CONNECTED to Transmitter!");
  } else {
    Serial.println("Connection FAILED. Check if Transmitter is powered on.");
  }
}

// void loop() {
//   // Check if we are connected
//   if (!SerialBT.connected()) {
//     static unsigned long lastRetry = 0;
//     if (millis() - lastRetry > 5000) { // Retry every 5 seconds
//       Serial.println("Connection lost. Retrying...");
//       SerialBT.connect(remoteName);
//       lastRetry = millis();
//     }
//     return;
//   }

//   // Process incoming data
//   if (SerialBT.available()) {
//     DateTime now = rtc.now();
//     String data = SerialBT.readStringUntil('\n');
//     data.trim();

//     if (data.length() > 0) {
//       // Create timestamp: HH:MM:SS.ms
//       char timestamp[15];
//       sprintf(timestamp, "%02d:%02d:%02d.%03lu", 
//               now.hour(), now.min(), now.sec(), (unsigned long)(millis() % 1000));

//       String logLine = String(timestamp) + "," + data;

//       // Log to SD
//       File file = SD.open("/log.csv", FILE_APPEND);
//       if (file) {
//         file.println(logLine);
//         file.close();
//       }

//       // Output to Serial Monitor
//       Serial.println(logLine);
//     }
//   }
// }