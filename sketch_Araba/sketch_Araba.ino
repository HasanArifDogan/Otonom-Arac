#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

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

// Bluetooth Tanımlamaları
SoftwareSerial BTSerial(2, 3); // RX, TX

// Motor Pin Tanımlamaları
const int rFront1 = 22;
const int rFront2 = 21;
const int rBack1 = 24;
const int rBack2 = 23;
const int renablePin1 = 20;
const int renablePin2 = 19;
const int lFront1 = 11;
const int lFront2 = 12;
const int lBack1 = 17;
const int lBack2 = 10;
const int lenablePin1 = 13;
const int lenablePin2 = 15;

// Pusula Değişkenleri
int16_t rawX, rawY, rawZ;
float Xm, Ym, Zm;
float rota, sapma = 0.115, rotaAcisi, rotaFiltrele;

// Joystick Değişkenleri
double leftJoystickX = 0.0;
double leftJoystickY = 0.0;
double rightJoystickX = 0.0;
double rightJoystickY = 0.0;

String line = "";
unsigned long lastTime;

void setup() {
  // Seri iletişim başlatma
  Serial.begin(9600);
  BTSerial.begin(9600);
  Serial.println("Sistem baslatiliyor...");

  // Motor pinleri ayarla
  pinMode(rFront1, OUTPUT);
  pinMode(rFront2, OUTPUT);
  pinMode(renablePin1, OUTPUT);
  pinMode(rBack1, OUTPUT);
  pinMode(rBack2, OUTPUT);
  pinMode(renablePin2, OUTPUT);
  pinMode(lFront1, OUTPUT);
  pinMode(lFront2, OUTPUT);
  pinMode(lenablePin1, OUTPUT);
  pinMode(lBack1, OUTPUT);
  pinMode(lBack2, OUTPUT);
  pinMode(lenablePin2, OUTPUT);

  // Pusula başlatma
  Wire.begin();
  lis2mdlBaslat();
  
  dur();
  Serial.println("Sistem hazir!");
}

void loop() {
  // Bluetooth veri kontrolü
  bluetoothKontrol();
  
  // Pusula verilerini oku ve işle
  pusulaOku();
  
  // Hareket kontrolü
  hareketKontrol();
}

// Bluetooth Kontrol Fonksiyonu
void bluetoothKontrol() {
  while (BTSerial.available()) {
    char message = BTSerial.read();
    Serial.print(message);

    switch (message) {
      case '\n':
        {
          String varName = line.substring(0, line.indexOf('=')),
                 varValue = line.substring(line.indexOf('=') + 1, line.length());

          // Switch durumlarını kontrol et
          /*if (varName == "switch1") {
            if(varValue == "true") belirliYoneGit(0);    // Kuzey (0°)
          }
          else if (varName == "switch2") {
            if(varValue == "true") belirliYoneGit(90);   // Doğu (90°)
          }
          else if (varName == "switch3") {
            if(varValue == "true") belirliYoneGit(180);  // Güney (180°)
          }
          else if (varName == "switch4") {
            if(varValue == "true") belirliYoneGit(270);  // Batı (270°)
          }*/

          // Joystick verilerini de işlemeye devam et
          if (varName == "leftJoystickX")
            leftJoystickX = varValue.toInt();
          else if (varName == "leftJoystickY")
            leftJoystickY = varValue.toInt();
          else if (varName == "rightJoystickX")
            rightJoystickX = varValue.toInt();
          else if (varName == "rightJoysticky")
            rightJoystickY = varValue.toInt();
          line = "";
          break;
        }

      default:
        {
          if (millis() - lastTime > 100)
            line = "";

          lastTime = millis();
          line = line + message;
          break;
        }
    }
  }
}

// Pusula Okuma Fonksiyonu
void pusulaOku() {
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
  //rotaFiltrele = rotaFiltrele * 0.85 + rotaAcisi * 0.15;
  rotaFiltrele = rotaFiltrele * 0.90 + rotaAcisi * 0.10; // Daha az filtreleme


  //icSicaklik = ikiByteOku(TEMP_OUT_L_REG);
  
  //Serial.print("Xm: "); Serial.print(Xm);
  //Serial.print(" Ym: "); Serial.println(Ym);
  //Serial.print(" Zm: "); Serial.println(Zm);

  yonYaz(rotaAcisi, 5.0);
  //rawXyzYazdir();
  delay(100);
}

// Hareket Kontrol Fonksiyonu
void hareketKontrol() {
  const int deadzone = 40; // Ölü bölge
  
  // İleri-Geri kontrolü (sadece leftJoystickY)
  if (abs(leftJoystickY) > deadzone) {
    if (leftJoystickY > 0) ileri();
    else geri();
    return;
  }
  
  // Sağ-Sol kontrolü (sadece rightJoystickX)
  if (abs(rightJoystickX) > deadzone) {
    if (rightJoystickX > 0) sagaDon();
    else solaDon();
    return;
  }
  
  // Joystickler nötr konumda
  dur();
}

/**************************************
 * PUSULA FONKSİYONLARI
 **************************************/
int lis2mdlBaslat(){
  Wire.beginTransmission(LIS2MDL_ADDRESS);
  Wire.write(CFG_REG_A);
  Wire.write(0x80); // 10 Hz mod
  Wire.write(CFG_REG_C);
  Wire.write(0x01); // Mag'yi etkinleştir
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
  } else {
    Serial.println("Veri alım hatası!");
    return 0;
  }

  int16_t result = (int16_t)(highByte << 8 | lowByte);
  return result;
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

void rawXyzYazdir(){
  Serial.print("rawX: "); Serial.println(rawX);
  Serial.print("rawY: "); Serial.println(rawY);
  Serial.print("rawZ: "); Serial.println(rawZ);
  Serial.println();
}

/**************************************
 * MOTOR FONKSİYONLARI
 **************************************/
/*
void belirliYoneGit(float derece){
  //Verilen dereceye göre Doğuya batı kuzey güney e dön ve ilerle
}*/

void ileri() {
  // Motor hızlarını ayarla
  analogWrite(renablePin1, 255); analogWrite(renablePin2, 255);
  analogWrite(lenablePin1, 255); analogWrite(lenablePin2, 255);

  // Tüm motorlar ileri
  digitalWrite(rFront1, HIGH); digitalWrite(rFront2, LOW);
  digitalWrite(rBack1, HIGH); digitalWrite(rBack2, LOW);
  digitalWrite(lFront1, HIGH); digitalWrite(lFront2, LOW);
  digitalWrite(lBack1, HIGH); digitalWrite(lBack2, LOW);
}

void geri() {
  // Motor hızlarını ayarla
  analogWrite(renablePin1, 255); analogWrite(renablePin2, 255);
  analogWrite(lenablePin1, 255); analogWrite(lenablePin2, 255);

  // Tüm motorlar geri
  digitalWrite(rFront1, LOW); digitalWrite(rFront2, HIGH);
  digitalWrite(rBack1, LOW); digitalWrite(rBack2, HIGH);
  digitalWrite(lFront1, LOW); digitalWrite(lFront2, HIGH);
  digitalWrite(lBack1, LOW); digitalWrite(lBack2, HIGH);
}

void sagaDon() {
  // Motor hızlarını ayarla
  analogWrite(renablePin1, 255); analogWrite(renablePin2, 255);
  analogWrite(lenablePin1, 255); analogWrite(lenablePin2, 255);

  // Sağ taraf motorlar TAM GERİ (fren etkisi)
  digitalWrite(rFront1, LOW); digitalWrite(rFront2, HIGH);
  digitalWrite(rBack1, LOW); digitalWrite(rBack2, HIGH);
  
  // Sol taraf motorlar TAM İLERİ
  digitalWrite(lFront1, HIGH); digitalWrite(lFront2, LOW);
  digitalWrite(lBack1, HIGH); digitalWrite(lBack2, LOW);
}

void solaDon() {
  // Motor hızlarını ayarla
  analogWrite(renablePin1, 255); analogWrite(renablePin2, 255);
  analogWrite(lenablePin1, 255); analogWrite(lenablePin2, 255);

  // Sağ taraf motorlar TAM İLERİ 
  digitalWrite(rFront1, HIGH); digitalWrite(rFront2, LOW);
  digitalWrite(rBack1, HIGH); digitalWrite(rBack2, LOW);
  
  // Sol taraf motorlar TAM Geri (fren etkisi)
  digitalWrite(lFront1, LOW); digitalWrite(lFront2, HIGH);
  digitalWrite(lBack1, LOW); digitalWrite(lBack2, HIGH);
}

void dur() {
  // Motor hızlarını sıfırla
  analogWrite(renablePin1, 0); analogWrite(renablePin2, 0);
  analogWrite(lenablePin1, 0); analogWrite(lenablePin2, 0);

  // Tüm motorları durdur
  digitalWrite(rFront1, LOW); digitalWrite(rFront2, LOW);
  digitalWrite(rBack1, LOW); digitalWrite(rBack2, LOW);
  digitalWrite(lFront1, LOW); digitalWrite(lFront2, LOW);
  digitalWrite(lBack1, LOW); digitalWrite(lBack2, LOW);
}