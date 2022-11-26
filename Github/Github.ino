#include <Wire.h>

// ADDRESS DARI DATASHEET
#define MPU 0x68
#define ACCEL 0x3B
#define GYRO 0x43

// SIMPAN DATA MENTAH
float AccRawX1, AccRawY1, AccRawZ1, GyRawX1, GyRawY1, GyRawZ1;

void setup() {
  // put your setup code here, to run once:
  mpu6050_setup();
}

void loop() {
  // put your main code here, to run repeatedly:
}

void mpu6050_setup() {
  // RESET MPU
  Wire.beginTransmission(MPU);
  Wire.write(MPU);
  Wire.write(0b00000111);
  Wire.endTransmission();
  // RESET BACK TO 0
  Wire.beginTransmission(0x68);
  Wire.write(MPU);
  Wire.write(0b00000000);
  Wire.endTransmission();
  // DLPF
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0b00000001);
  Wire.endTransmission();
  // POWER MANAGEMENT ENABLE TEMP
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  // Konfigurasi akselerometer dengan full scale range 2g + DHPF
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0b00000111);
  Wire.endTransmission();
  // Konfigurasi gyroscope dengan full scale range 250 deg/s
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void read_mpu6050_accel() {
  Wire.beginTransmission(MPU);
  Wire.write(ACCEL);
  Wire.endTransmission();
  // MULAI BACA 6 BYTES
  Wire.requestFrom(MPU, 6);
  // SIMPAN SETIAP PEMBACAAN 8 BIT MSB DAN LSB
  AccRawX1 = Wire.read() << 8 | Wire.read(); 
  AccRawY1 = Wire.read() << 8 | Wire.read();  
  AccRawZ1 = Wire.read() << 8 | Wire.read();  
}

void read_mpu6050_gyro() {
  Wire.beginTransmission(MPU);
  Wire.write(GYRO);
  Wire.endTransmission();
  Wire.requestFrom(MPU, 6);
  GyRawX1 = Wire.read() << 8 | Wire.read();
  GyRawY1 = Wire.read() << 8 | Wire.read();
  GyRawZ1 = Wire.read() << 8 | Wire.read();
}
