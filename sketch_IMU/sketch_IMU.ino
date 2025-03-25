#include <Wire.h> // I2C Arduino Kütüphanesi

#define LSM6DS3_ADDRESS 0x6A

// Register tanımlamaları
#define CTRL1_XL 0x10
#define CTRL2_G  0x11
#define CTRL7_G  0x16
#define CTRL8_XL 0x17

#define OUTX_L_G 0x22
#define OUTX_H_G 0x23
#define OUTY_L_G 0x24
#define OUTY_H_G 0x25
#define OUTZ_L_G 0x26
#define OUTZ_H_G 0x27

#define OUTX_L_XL 0x28
#define OUTX_H_XL 0x29
#define OUTY_L_XL 0x2A
#define OUTY_H_XL 0x2B
#define OUTZ_L_XL 0x2C
#define OUTZ_H_XL 0x2D

// Kalman filtre değişkenleri
float kalmanX = 0, kalmanP = 1, kalmanQ = 0.01, kalmanR = 0.1;
float velocityX = 0, positionX = 0;
float offsetX = 0;  

unsigned long prevTime, currentTime;

// Kalman filtresi fonksiyonu
float kalmanFilter(float input) {
  kalmanP += kalmanQ;
  float K = kalmanP / (kalmanP + kalmanR);
  kalmanX += K * (input - kalmanX);
  kalmanP *= (1 - K);
  return kalmanX;
}

// Register yazma fonksiyonu
void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(LSM6DS3_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// 16 bit veri okuma fonksiyonu
int16_t read16(uint8_t reg_l, uint8_t reg_h) {
  Wire.beginTransmission(LSM6DS3_ADDRESS);
  Wire.write(reg_l);
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DS3_ADDRESS, 2);
  int16_t value = 0;
  if (Wire.available() >= 2) {
    uint8_t lsb = Wire.read();
    uint8_t msb = Wire.read();
    value = (int16_t)(msb << 8 | lsb);
  }
  return value;
}

// İvmeölçer verilerini oku
void readAccelerometer(float &ax, float &ay, float &az) {
  int16_t rawX = read16(OUTX_L_XL, OUTX_H_XL);
  int16_t rawY = read16(OUTY_L_XL, OUTY_H_XL);
  int16_t rawZ = read16(OUTZ_L_XL, OUTZ_H_XL);

  float accelScale = 0.122 / 1000.0; // ±4g ölçek faktörü (g cinsinden)
  ax = rawX * accelScale;
  ay = rawY * accelScale;
  az = rawZ * accelScale;
}

// Jiroskop verilerini oku
void readGyroscope(float &gx, float &gy, float &gz) {
  int16_t rawX = read16(OUTX_L_G, OUTX_H_G);
  int16_t rawY = read16(OUTY_L_G, OUTY_H_G);
  int16_t rawZ = read16(OUTZ_L_G, OUTZ_H_G);

  float gyroScale = 70.0 / 1000.0; // 2000 dps için ölçek faktörü (dps cinsinden)
  gx = rawX * gyroScale;
  gy = rawY * gyroScale;
  gz = rawZ * gyroScale;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Sensör yapılandırması
  writeReg(CTRL2_G, 0x4C);  // Jiroskop: 104 Hz, 2000 dps
  writeReg(CTRL1_XL, 0x4A); // İvmeölçer: 104 Hz, ±4g
  writeReg(CTRL7_G, 0x00);  // Jiroskop yüksek performans
  writeReg(CTRL8_XL, 0x09); // İvmeölçer LPF: ODR/4

  delay(300);

  // Başlangıç ofsetini hesapla
  float sum = 0;
  for (int i = 0; i < 500; i++) {
    float ax, ay, az;
    readAccelerometer(ax, ay, az);
    sum += ax;
    delay(2);
  }
  offsetX = sum / 500;
  kalmanX = offsetX;

  Serial.print("Başlangıç Ofseti: ");
  Serial.println(offsetX, 6);

  prevTime = micros();
}

void loop() {
  currentTime = micros();
  float dt = (currentTime - prevTime) / 1e6; // Saniye cinsinden zaman farkı

  // Sensör verilerini oku
  float ax, ay, az, gx, gy, gz;
  readAccelerometer(ax, ay, az);
  readGyroscope(gx, gy, gz);

  // Kalman filtresi uygula ve ivme hesapla
  float filteredAcceleration = kalmanFilter(ax);
  float accelerationX = filteredAcceleration - offsetX;

  // Gürültü filtresi
  if (abs(accelerationX) < 0.0) {
    accelerationX = 0;
  }

  // Hız ve pozisyon hesapla
  velocityX += accelerationX * dt;
  if (accelerationX == 0) {
    velocityX *= 0.90;
    if (abs(velocityX) < 0.02) velocityX = 0;
  }
  positionX += velocityX * dt;

  // Seri port çıktısı
  
  Serial.print("dt (s): ");
  Serial.print(dt, 6);
  Serial.print(" | AccX (g): ");
  Serial.print(accelerationX, 6);
  Serial.print(" | VelX (m/s): ");
  Serial.print(velocityX, 6);
  Serial.print(" | PosX (m): ");
  Serial.print(positionX, 6);
  Serial.print(" | Gyro (dps) -> X: ");
  Serial.print(gx, 6);
  Serial.print(" | Y: ");
  Serial.print(gy, 6);
  Serial.print(" | Z: ");
  Serial.println(gz, 6);

  prevTime = currentTime;
  delay(100);
}
