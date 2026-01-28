#include <Wire.h>

#define MPU_ADDR 0x68   // MPU6050 I2C address

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

void setup() {
  Serial.begin(115200);

  // ESP32 I2C pins
  Wire.begin(21, 22);   // SDA, SCL

  setupMPU();
}

void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  printData();
  delay(50);
}


void setupMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);        // Power management register
  Wire.write(0x00);        // Wake up MPU6050
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);        // Gyro config
  Wire.write(0x00);        // ±250 deg/s
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);        // Accel config
  Wire.write(0x00);        // ±2g
  Wire.endTransmission();
}

void recordAccelRegisters() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 6, true);

  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();

  processAccelData();
}

void processAccelData() {
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 6, true);

  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();

  processGyroData();
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0;
  rotZ = gyroZ / 131.0;
}

void printData() {
  Serial.print("Gyro (deg/s): ");
  Serial.print("X=");
  Serial.print(rotX);
  Serial.print("  Y=");
  Serial.print(rotY);
  Serial.print("  Z=");
  Serial.print(rotZ);

  Serial.print(" | Accel (g): ");
  Serial.print("X=");
  Serial.print(gForceX);
  Serial.print("  Y=");
  Serial.print(gForceY);
  Serial.print("  Z=");
  Serial.println(gForceZ);
}