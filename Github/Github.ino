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
