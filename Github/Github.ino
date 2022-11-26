void setup() {
  // put your setup code here, to run once:
  mpu6050_setup();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void mpu6050_setup() {
  Wire.beginTransmission(MPU);
  Wire.write(MPU);
  Wire.write(0b00000111);
  Wire.endTransmission();
  //mpu reset
  Wire.beginTransmission(0x68);
  Wire.write(MPU);
  Wire.write(0b00000000);
  Wire.endTransmission();
  // low pass filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0b00000001);
  Wire.endTransmission();
  // powe management / reset
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  // acelero dan high pass filter
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0b00000111);
  Wire.endTransmission();
  // gyro reset
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
}