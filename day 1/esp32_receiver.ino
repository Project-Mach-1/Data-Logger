/*
 * ESP32 RECEIVER - Smart Helmet Data Logger
 * This ESP32 receives MPU6050 sensor data via Bluetooth Classic
 * and stores it to an SD card
 * 
 * Hardware Connections:
 * SD Card Module -> ESP32
 * CS   -> GPIO5
 * MOSI -> GPIO23
 * MISO -> GPIO19
 * SCK  -> GPIO18
 * VCC  -> 5V (or 3.3V depending on module)
 * GND  -> GND
 */

#include "BluetoothSerial.h"
#include <SD.h>
#include <SPI.h>

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

// Bluetooth Serial object
BluetoothSerial SerialBT;

// SD Card pins (default SPI pins for ESP32)
#define SD_CS_PIN 5
#define SD_MOSI_PIN 23
#define SD_MISO_PIN 19
#define SD_SCK_PIN 18

// LED pins for status indication
const int LED_BT = 2;      // Bluetooth connection status
const int LED_SD = 4;      // SD card write status

// File handling
File dataFile;
String fileName = "/helmet_data.csv";
unsigned long fileStartTime = 0;
const unsigned long FILE_ROTATION_TIME = 3600000; // Create new file every hour (in ms)

// Buffer for incoming data
String dataBuffer = "";
unsigned long lastDataTime = 0;
const unsigned long DATA_TIMEOUT = 5000; // 5 seconds timeout

// Statistics
unsigned long recordCount = 0;
unsigned long errorCount = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Receiver - Smart Helmet Data Logger");
  
  // Initialize LED pins
  pinMode(LED_BT, OUTPUT);
  pinMode(LED_SD, OUTPUT);
  digitalWrite(LED_BT, LOW);
  digitalWrite(LED_SD, LOW);
  
  // Initialize SD card
  Serial.println("Initializing SD card...");
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    while (1) {
      digitalWrite(LED_SD, !digitalRead(LED_SD));
      delay(200); // Fast blink indicates SD error
    }
  }
  
  Serial.println("SD card initialized successfully");
  digitalWrite(LED_SD, HIGH);
  
  // Print SD card info
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  
  // Create new data file with header
  createNewFile();
  
  // Initialize Bluetooth
  Serial.println("Initializing Bluetooth...");
  SerialBT.begin("ESP32_Receiver", true); // true = master mode
  Serial.println("Bluetooth started in master mode");
  
  // Connect to transmitter
  Serial.println("Connecting to ESP32_Helmet...");
  bool connected = SerialBT.connect("ESP32_Helmet");
  
  if (connected) {
    Serial.println("Connected to ESP32_Helmet!");
    digitalWrite(LED_BT, HIGH);
  } else {
    Serial.println("Failed to connect!");
    digitalWrite(LED_BT, LOW);
  }
  
  Serial.println("System ready. Waiting for data...");
}

void loop() {
  // Check Bluetooth connection status
  if (SerialBT.connected()) {
    digitalWrite(LED_BT, HIGH);
    
    // Read incoming data
    while (SerialBT.available()) {
      char c = SerialBT.read();
      
      if (c == '\n') {
        // Complete line received, process it
        if (dataBuffer.length() > 0) {
          processData(dataBuffer);
          dataBuffer = "";
        }
      } else if (c != '\r') {
        // Add character to buffer (ignore carriage returns)
        dataBuffer += c;
      }
      
      lastDataTime = millis();
    }
  } else {
    digitalWrite(LED_BT, LOW);
    // Try to reconnect
    static unsigned long lastReconnect = 0;
    if (millis() - lastReconnect > 5000) {
      Serial.println("Attempting to reconnect...");
      SerialBT.connect("ESP32_Helmet");
      lastReconnect = millis();
    }
  }
  
  // Check if we need to rotate the file
  if (millis() - fileStartTime > FILE_ROTATION_TIME) {
    closeFile();
    createNewFile();
  }
  
  // Check for data timeout
  if (lastDataTime > 0 && (millis() - lastDataTime > DATA_TIMEOUT)) {
    Serial.println("Warning: No data received for 5 seconds");
    lastDataTime = 0; // Reset to avoid repeated warnings
  }
  
  delay(1);
}

void createNewFile() {
  // Generate filename with timestamp
  unsigned long timestamp = millis();
  fileName = "/helmet_" + String(timestamp) + ".csv";
  
  Serial.print("Creating new file: ");
  Serial.println(fileName);
  
  // Open file in write mode
  dataFile = SD.open(fileName.c_str(), FILE_WRITE);
  
  if (dataFile) {
    // Write CSV header
    dataFile.println("Timestamp,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Temperature");
    dataFile.flush();
    Serial.println("File created with header");
    fileStartTime = millis();
    recordCount = 0;
  } else {
    Serial.println("Error creating file!");
    errorCount++;
  }
}

void closeFile() {
  if (dataFile) {
    dataFile.close();
    Serial.print("File closed. Records written: ");
    Serial.println(recordCount);
  }
}

void processData(String data) {
  // Blink SD LED to indicate write activity
  digitalWrite(LED_SD, !digitalRead(LED_SD));
  
  // Write to SD card
  if (dataFile) {
    dataFile.println(data);
    dataFile.flush(); // Ensure data is written immediately
    recordCount++;
    
    // Print to serial for debugging (every 10 records)
    if (recordCount % 10 == 0) {
      Serial.print("Records: ");
      Serial.print(recordCount);
      Serial.print(" | Last: ");
      Serial.println(data);
    }
  } else {
    Serial.println("Error: File not open!");
    errorCount++;
    // Try to reopen the file
    dataFile = SD.open(fileName.c_str(), FILE_APPEND);
  }
  
  digitalWrite(LED_SD, HIGH);
}

// Print system statistics every minute
void printStats() {
  static unsigned long lastStats = 0;
  if (millis() - lastStats > 60000) {
    Serial.println("\n=== System Statistics ===");
    Serial.print("Records written: ");
    Serial.println(recordCount);
    Serial.print("Errors: ");
    Serial.println(errorCount);
    Serial.print("Current file: ");
    Serial.println(fileName);
    Serial.print("SD card space used: ");
    Serial.print(SD.usedBytes() / 1024);
    Serial.println(" KB");
    Serial.println("========================\n");
    lastStats = millis();
  }
}
