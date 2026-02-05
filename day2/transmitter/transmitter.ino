#include "BluetoothSerial.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;

unsigned long lastTime = 0;
const int interval = 10; // 10ms = 100Hz

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_IMU_Transmitter"); // Bluetooth device name
  
  if (!mpu.begin()) {
    while (1) yield(); 
  }
  
  // Set accelerometer range (optional adjustment)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
}

void loop() {
  if (millis() - lastTime >= interval) {
    lastTime = millis();

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Format: "ax,ay,az"
    SerialBT.print(a.acceleration.x);
    SerialBT.print(",");
    SerialBT.print(a.acceleration.y);
    SerialBT.print(",");
    SerialBT.println(a.acceleration.z);
  }
}