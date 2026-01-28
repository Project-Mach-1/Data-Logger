/*
 * ESP32 TRANSMITTER - Smart Helmet Data Logger
 * This ESP32 reads MPU6050 sensor data and transmits it via Bluetooth Classic
 * to the receiver ESP32 for SD card storage
 * 
 * Hardware Connections:
 * MPU6050 -> ESP32
 * VCC -> 3.3V
 * GND -> GND
 * SCL -> GPIO22 (default I2C clock)
 * SDA -> GPIO21 (default I2C data)
 */

#include <Wire.h>
#include <MPU6050.h>
#include "BluetoothSerial.h"

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

// Bluetooth Serial object
BluetoothSerial SerialBT;

// MPU6050 object
MPU6050 mpu;

// Data structure for sensor readings
struct SensorData {
  unsigned long timestamp;
  int16_t accelX, accelY, accelZ;
  int16_t gyroX, gyroY, gyroZ;
  int16_t temperature;
};

SensorData data;

// Timing variables
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL = 100; // Send data every 100ms (10Hz)

// LED for status indication
const int LED_PIN = 2;
bool bluetoothConnected = false;

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Transmitter - Smart Helmet Data Logger");
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  
  // Check MPU6050 connection
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed!");
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200); // Fast blink indicates error
    }
  }
  
  // Configure MPU6050
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  // ±2g
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);   // ±250°/s
  
  // ====================================================================
  // CALIBRATION OFFSETS - Replace with your values!
  // ====================================================================
  // Run the "mpu6050_calibration.ino" sketch first to get these values
  // Then copy them here:
  
  mpu.setXAccelOffset(0);  // Replace 0 with your calibrated value
  mpu.setYAccelOffset(0);  // Replace 0 with your calibrated value
  mpu.setZAccelOffset(0);  // Replace 0 with your calibrated value
  mpu.setXGyroOffset(0);   // Replace 0 with your calibrated value
  mpu.setYGyroOffset(0);   // Replace 0 with your calibrated value
  mpu.setZGyroOffset(0);   // Replace 0 with your calibrated value
  
  // Example (your values will be different):
  // mpu.setXAccelOffset(-2456);
  // mpu.setYAccelOffset(1589);
  // mpu.setZAccelOffset(1234);
  // mpu.setXGyroOffset(45);
  // mpu.setYGyroOffset(-23);
  // mpu.setZGyroOffset(12);
  // ====================================================================
  
  Serial.println("Calibration offsets applied");
  
  // Initialize Bluetooth
  Serial.println("Initializing Bluetooth...");
  SerialBT.begin("ESP32_Helmet"); // Bluetooth device name
  Serial.println("Bluetooth started. Device name: ESP32_Helmet");
  Serial.println("Waiting for connection...");
  
  // Register callback for connection status
  SerialBT.register_callback(btCallback);
}

void loop() {
  // Check if it's time to send data
  unsigned long currentTime = millis();
  
  if (currentTime - lastSendTime >= SEND_INTERVAL) {
    lastSendTime = currentTime;
    
    // Read sensor data
    readSensorData();
    
    // Send data via Bluetooth if connected
    if (bluetoothConnected) {
      sendData();
      digitalWrite(LED_PIN, HIGH); // LED on when transmitting
    } else {
      digitalWrite(LED_PIN, LOW);
    }
    
    // Print to Serial for debugging
    printData();
  }
  
  // Small delay to prevent watchdog issues
  delay(1);
}

void readSensorData() {
  // Get timestamp
  data.timestamp = millis();
  
  // Read accelerometer and gyroscope
  mpu.getAcceleration(&data.accelX, &data.accelY, &data.accelZ);
  mpu.getRotation(&data.gyroX, &data.gyroY, &data.gyroZ);
  
  // Read temperature
  data.temperature = mpu.getTemperature();
}

void sendData() {
  // Send data as comma-separated values with newline terminator
  // Format: timestamp,ax,ay,az,gx,gy,gz,temp
  SerialBT.print(data.timestamp);
  SerialBT.print(",");
  SerialBT.print(data.accelX);
  SerialBT.print(",");
  SerialBT.print(data.accelY);
  SerialBT.print(",");
  SerialBT.print(data.accelZ);
  SerialBT.print(",");
  SerialBT.print(data.gyroX);
  SerialBT.print(",");
  SerialBT.print(data.gyroY);
  SerialBT.print(",");
  SerialBT.print(data.gyroZ);
  SerialBT.print(",");
  SerialBT.println(data.temperature);
}

void printData() {
  // Print to serial monitor for debugging
  Serial.print("Time: ");
  Serial.print(data.timestamp);
  Serial.print(" | Accel X:");
  Serial.print(data.accelX);
  Serial.print(" Y:");
  Serial.print(data.accelY);
  Serial.print(" Z:");
  Serial.print(data.accelZ);
  Serial.print(" | Gyro X:");
  Serial.print(data.gyroX);
  Serial.print(" Y:");
  Serial.print(data.gyroY);
  Serial.print(" Z:");
  Serial.print(data.gyroZ);
  Serial.print(" | Temp:");
  Serial.print(data.temperature / 340.0 + 36.53); // Convert to Celsius
  Serial.print("°C");
  Serial.print(" | BT: ");
  Serial.println(bluetoothConnected ? "Connected" : "Disconnected");
}

void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    Serial.println("Bluetooth client connected!");
    bluetoothConnected = true;
  } else if (event == ESP_SPP_CLOSE_EVT) {
    Serial.println("Bluetooth client disconnected!");
    bluetoothConnected = false;
  }
}
