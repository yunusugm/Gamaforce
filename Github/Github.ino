#include <Wire.h>

// ADDRESS DARI DATASHEET
#define MPU 0x68
#define ACCEL 0x3B
#define GYRO 0x43

// SIMPAN DATA MENTAH
float AccRawX1, AccRawY1, AccRawZ1, GyRawX1, GyRawY1, GyRawZ1;
// KALIBRASI VAR
float AccRawXc, AccRawYc, AccRawZc, GyRawXc, GyRawYc, GyRawZc;
float AccRawX2, AccRawY2, AccRawZ2, GyRawX2, GyRawY2, GyRawZ2;
float AccAngleX, AccAngleY, AccAngleZ, GyAngleX, GyAngleY, GyAngleZ;
float roll, pitch;
float mpuTimer, mpuDTime;

void setup() {
  // put your setup code here, to run once:
  mpu6050_setup();
  calibrate_mpu6050();
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

void calibrate_mpu6050() {
  Serial.print("calibrating");

  for (int i = 0; i < 2000; i++) {
    read_mpu6050_accel();
    read_mpu6050_gyro();
    AccRawXc += AccRawX1;
    AccRawYc += AccRawY1;
    AccRawZc += AccRawZ1;
    GyRawXc += GyRawX1;
    GyRawYc += GyRawY1;
    GyRawZc += GyRawZ1;

    if (i % 100 == 0) Serial.print(".");
  }
  // DIRATA-RATA
  AccRawXc /= 2000;
  AccRawYc /= 2000;
  AccRawZc /= 2000;
  GyRawXc /= 2000;
  GyRawYc /= 2000;
  GyRawZc /= 2000;
  Serial.println(AccRawXc);
  Serial.println(AccRawYc);
  Serial.println(AccRawZc);
}

void calculate_angle() {
  read_mpu6050_accel();

  AccRawX2 = (AccRawX1 + (-1 * AccRawXc)) / 8192.0;
  AccRawY2 = (AccRawY1 + (-1 * AccRawYc)) / 8192.0;
  AccRawZ2 = (AccRawZ1 + (8192 - AccRawZc)) / 8192.0;
  // AccRawX2 = (AccRawX1 + (-1 * AccRawXc)) / 16384.0;
  // AccRawY2 = (AccRawY1 + (-1 * AccRawYc)) / 16384.0;
  // AccRawZ2 = (AccRawZ1 + (16384 - AccRawZc)) / 16384.0;


  AccAngleX = (atan2(AccRawX2, AccRawZ2)) * 57.2957795;
  AccAngleY = (atan2(AccRawY2, AccRawZ2)) * 57.2957795;

  for (int i = 0; i < 250; i++) {
    mpuDTime = (millis() - mpuTimer) / 1000;
    read_mpu6050_gyro();

    GyRawX2 = GyRawX1 / 65.5;
    GyRawY2 = GyRawY1 / 65.5;
    // GyRawX2 = GyRawX1 / 131.0;
    // GyRawY2 = GyRawY1 / 131.0;
    // GyRawZ2 = (GyRawZ1 + (-1 * GyRawXc)) / 65.5;

    GyAngleX += GyRawX2 * mpuDTime;
    GyAngleY += GyRawY2 * mpuDTime;
  }
  GyAngleX = (GyAngleX + (-1 * GyRawXc)) / 250;
  GyAngleY = (GyAngleY + (-1 * GyRawYc)) / 250;

  roll = 0.9998 * AccAngleX + 0.0002 * GyAngleX;
  pitch = 0.9998 * AccAngleY + 0.0002 * GyAngleY;
}