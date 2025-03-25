#include <Wire.h> //I2C Arduino Kütüphanesi

// LIS2MDL I2C varsayılan slave addresi 
#define LIS2MDL_ADDRESS 0x1E

// Hard-Iron Offset Kaydedicileri
#define OFFSET_X_REG_L 0x45
#define OFFSET_X_REG_H 0x46
#define OFFSET_Y_REG_L 0x47
#define OFFSET_Y_REG_H 0x48
#define OFFSET_Z_REG_L 0x49
#define OFFSET_Z_REG_H 0x4A

// Konfigürasyon Kaydedicileri
#define CFG_REG_A 0x60
#define CFG_REG_B 0x61
#define CFG_REG_C 0x62

// Durum Kaydedicisi
#define STATUS_REG 0x67

// Çıkış Kaydedicileri
#define OUTX_L_REG 0x68
#define OUTX_H_REG 0x69
#define OUTY_L_REG 0x6A
#define OUTY_H_REG 0x6B
#define OUTZ_L_REG 0x6C
#define OUTZ_H_REG 0x6D

// Sensör İç Sıcaklık Kaydedicileri
#define TEMP_OUT_L_REG 0x6E
#define  TEMP_OUT_H_REG 0x6F

// Okunan değerlerin atanacağı 16 bitlik değişkenler.
int16_t rawX; 
int16_t rawY; 
int16_t rawZ;
int16_t icSicaklik;

float Xm, Ym, Zm;

float rota, sapma, rotaAcisi, rotaFiltrele;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  lis2mdlBaslat();
}

void loop() {
  //verilerHazirMi();
  
  rawX = ikiByteOku(OUTX_L_REG);
  // Datasheet Hassasiyeti: 1.5 mG/digit
  Xm = rawX * 0.00015;  // Gauss unit

  rawY = ikiByteOku(OUTY_L_REG);
  Ym = rawY * 0.00015; 

  rawZ = ikiByteOku(OUTZ_L_REG);
  Zm = rawZ * 0.00015;

  // Konuma göre rotayı düzeltmek lazım
  // Lokasyona göre sapmayı burdan bul. https://www.ngdc.noaa.gov/geomag-web/ 
  // Bende 6.6 derece. Yani 0.115 rad
  rota = atan2(Ym, Xm);
  sapma = 0.115;
  rota += sapma;

  // Negatid olduğunda düzeltme
  if(rota <= 0) rota += 2*PI;

  // Sapma eklediğimizden dolayı düzeltme
  if(rota >= 2*PI) rota -= 2*PI;
  
  rotaAcisi = rota * 180/PI;  //radyandan dereceye[0-360]

  // Çıkış açısını düzeltme / Low Pass filtresi
  // Ani dalgalanmaları yumşatır
  rotaFiltrele = rotaFiltrele * 0.85 + rotaAcisi * 0.15;


  icSicaklik = ikiByteOku(TEMP_OUT_L_REG);

  yonYaz(rotaAcisi, 5.0);
  //xyzYazdir();

  delay(500);
}

int lis2mdlBaslat(){
  Wire.beginTransmission(LIS2MDL_ADDRESS);
  Wire.write(CFG_REG_A);  // Veri yazılacak kaydedici adresini seç.
  Wire.write(0x80);       // 0x80h datasını yolla. Mag = 10 Hz (yüksek çözünürlüklü ve sürekli mod) 

  Wire.write(CFG_REG_C);  // Veri yazılacak kaydedici adresini seç.
  Wire.write(0x01);  // 0x01h datasını yolla. Mag'yi etkinleştirir. Veriye hazır interrupt. 
  return Wire.endTransmission();
}

int16_t ikiByteOku(uint8_t lowAddress){
  uint8_t lowByte, highByte;
  Wire.beginTransmission(LIS2MDL_ADDRESS);
  Wire.write(lowAddress);
  Wire.endTransmission();
  Wire.requestFrom(LIS2MDL_ADDRESS, 2);
  
  if (Wire.available() >= 2) {
    lowByte = Wire.read();
    highByte = Wire.read();
  }

  return (int16_t)(highByte << 8 | lowByte);
}

void xyzYazdir(){
  Serial.print("  MagX: "); Serial.print(rawX);
  Serial.print("  MagY: "); Serial.print(rawY);
  Serial.print("  MagZ: "); Serial.println(rawZ);
  Serial.println();
}

void icSicaklikYaz(){
  float sicaklikC = icSicaklik / 8.0; // LIS2MDL sıcaklığı 8 LSB/°C olarak verir.
  Serial.print(" Sensor Temperature (°C): "); Serial.println(sicaklikC, 2);
  Serial.println();
}

void yonYaz(float pusulaAci, float pay){
  Serial.println();
  Serial.print("Derece: "); Serial.print(pusulaAci);
  Serial.println();
  if(pusulaAci > 360 - pay || pusulaAci < pay){
    Serial.println("Kuzeye Gidiliyor.");
  }
  else if(pusulaAci > 90 - pay && pusulaAci < 90 + pay){
    Serial.println("Batiya Gidiliyor.");
  }
  else if(pusulaAci > 180 - pay && pusulaAci < 180 + pay){
    Serial.println("Guneye Gidiliyor.");
  }
  else if(pusulaAci > 270 - pay && pusulaAci < 270 + pay){
    Serial.println("Doguya Gidiliyor.");
  }
}

//endTranssmission() kontrolü
/*
void aktarmaKontrol(int t){
  switch(t){
    case 0: Serial.println("Transsmisson: Success.");
    case 1: Serial.println("Transsmisson: Data too long to fit in transmit buffer.");
    case 2: Serial.println("Transsmisson: Received NACK on transmit of address.");
    case 3: Serial.println("Transsmisson: Received NACK on transmit of data.");
    case 4: Serial.println("Transsmisson: Other error.");
    case 5: Serial.println("Transsmisson: Timeout");
  }
}
*/

/*

int8_t verilerHazirMi(){
  Wire.beginTransmission(LIS2MDL_ADDRESS);
  Wire.write(STATUS_REG); 
  Wire.endTransmission(); 
  int8_t status;
  Wire.requestFrom(LIS2MDL_ADDRESS, 1);
  if (Wire.available()) {
    status = Wire.read(); 

   // Durumu kontrol et
   if (status & 0x08) {  // Z ekseni veri hazır mı?
     Serial.println("Z ekseni verisi hazır.");
    }
   if (status & 0x04) {  // Y ekseni veri hazır mı?
      Serial.println("Y ekseni verisi hazır.");
    }
    if (status & 0x02) {  // X ekseni veri hazır mı?
     Serial.println("X ekseni verisi hazır.");
    }
    if (status & 0x10) {  // Sıcaklık verisi hazır mı?
      Serial.println("Sıcaklık verisi hazır.");
    }
  }
  return status;
}
*/

