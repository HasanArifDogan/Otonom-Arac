#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

// LIS2MDL I2C varsayılan slave addresi 
#define LIS2MDL_ADDRESS 0x1E

// Çıkış Kaydedicileri
#define OUTX_L_REG 0x68
#define OUTX_H_REG 0x69
#define OUTY_L_REG 0x6A
#define OUTY_H_REG 0x6B
#define OUTZ_L_REG 0x6C
#define OUTZ_H_REG 0x6D

// Konfigürasyon Kaydedicileri
#define CFG_REG_A 0x60
#define CFG_REG_C 0x62

// Sensör Değişkenleri
int16_t rawX, rawY, rawZ;
float Xm, Ym, Zm;

// Kalibrasyon değişkenleri
float magXmin, magXmax, magYmin, magYmax, magZmin, magZmax;
float magXoffset, magYoffset, magZoffset;
float magXscale, magYscale, magZscale;
bool kalibrasyon_yapildi = false;

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

// Joystick Değişkenleri
double leftJoystickX = 0.0;
double leftJoystickY = 0.0;
double rightJoystickX = 0.0;
double rightJoystickY = 0.0;

//Pusula Değişkenleri
float heading;

//Otonom Sürüş Değişkenleri
bool dogruYoneDondu = false;

//sonar sensor
const int trigPin = 6;
const int echoPin = 7;
const float SES_HIZI = 0.03432; //cm/μS cinsinden sesin hızı 0.03432 cm/μS
unsigned long sure;
float uzaklik;

String line = "";
unsigned long lastTime;

unsigned long delayStart;
bool delayRunning = false;

void setup() {
  // Seri iletişim başlatma
  Serial.begin(9600);
  BTSerial.begin(9600);
  //Serial.println("Sistem baslatiliyor...");

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

  //Sonar sensör pinler
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Pusula başlatma
  Wire.begin();
  lis2mdlBaslat();
  
  dur();
  delayStart = millis();
  //Serial.println("Sistem hazir!");
}

void loop() {
  // Bluetooth veri kontrolü
  bluetoothKontrol();
  
  // Pusula verilerini oku ve işle
  pusulaOku();
  
  // Hareket kontrolü
  hareketKontrol();

  //Çarpma Önleme
  carpmaOnle();
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
          if (varName == "switch1") {
            if(varValue == "true") belirliYoneGit(0,3);   // Kuzey (0°), 3sn   
          }
          else if (varName == "switch2") {
            if(varValue == "true") belirliYoneGit(1,3);   // Batı (90°), 3sn
          }
          else if (varName == "switch3") {
            if(varValue == "true") belirliYoneGit(2,3);   // Güney (180°), 3sn
          }
          else if (varName == "switch4") {
            if(varValue == "true") belirliYoneGit(3,3);   // Doğu (270°), 3sn
          }

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
float pusulaOku() {
// Ham verileri oku
  rawX = ikiByteOku(OUTX_L_REG);
  rawY = ikiByteOku(OUTY_L_REG);
  rawZ = ikiByteOku(OUTZ_L_REG);
  
  // Gauss birimine çevir
  Xm = rawX * 0.00015;
  Ym = rawY * 0.00015;
  Zm = rawZ * 0.00015;

  if (kalibrasyon_yapildi) {
    // Kalibre edilmiş değerleri hesapla
    Xm = (Xm - magXoffset) * magXscale;
    Ym = (Ym - magYoffset) * magYscale;
    Zm = (Zm - magZoffset) * magZscale;
  }
  
  // Değerleri yazdır
  Serial.print("X: "); Serial.print(Xm);
  Serial.print(" Y: "); Serial.print(Ym);
  Serial.print(" Z: "); Serial.println(Zm);
  
  // Pusula açısını hesapla ve yazdır (yatay düzlemde)
  heading = atan2(Ym, Xm) * 180.0 / PI;
  if (heading < 0) heading += 360.0;
  
  // Açıyı saat yönünde artacak şekilde düzelt
  //heading = 360.0 - heading;
  
  Serial.print("Açı: "); Serial.print(heading);
  Serial.print("° Yön: "); Serial.println(yonBul(heading));
  return heading;
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
    if (rightJoystickX > 0) sagaDon(255);
    else solaDon(255);
    return;
  }
  
  // Joystickler nötr konumda
  dur();
}

/**************************************
 * PUSULA FONKSİYONLARI
 **************************************/
void lis2mdlBaslat(){
  Wire.beginTransmission(LIS2MDL_ADDRESS);
  Wire.write(CFG_REG_A);
  Wire.write(0x80); // 10 Hz mod
  Wire.write(CFG_REG_C);
  Wire.write(0x01); // Mag'yi etkinleştir
  Wire.endTransmission();
  
  Serial.println("Kalibrasyon başlıyor...");
  Serial.println("Sensörü yavaşça tüm yönlerde döndürün (30 saniye)");
  kalibreEt();
  Serial.println("Kalibrasyon tamamlandı!");
  delay(1000);
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

void kalibreEt() {
  magXmin = magYmin = magZmin = 99999;
  magXmax = magYmax = magZmax = -99999;
  
  uint32_t baslangic = millis();
  
  // 30 saniye boyunca veri topla
  while (millis() - baslangic < 30000) {
    rawX = ikiByteOku(OUTX_L_REG);
    rawY = ikiByteOku(OUTY_L_REG);
    rawZ = ikiByteOku(OUTZ_L_REG);
    
    float X = rawX * 0.00015;
    float Y = rawY * 0.00015;
    float Z = rawZ * 0.00015;
    
    // Min/max değerleri güncelle
    if (X < magXmin) magXmin = X;
    if (X > magXmax) magXmax = X;
    if (Y < magYmin) magYmin = Y;
    if (Y > magYmax) magYmax = Y;
    if (Z < magZmin) magZmin = Z;
    if (Z > magZmax) magZmax = Z;
    
    delay(100);
  }
  
  // Offset ve scale değerlerini hesapla
  magXoffset = (magXmax + magXmin) / 2;
  magYoffset = (magYmax + magYmin) / 2;
  magZoffset = (magZmax + magZmin) / 2;
  
  magXscale = 1.0;
  magYscale = 1.0;
  magZscale = 1.0;
  
  float avg_delta_x = (magXmax - magXmin) / 2;
  float avg_delta_y = (magYmax - magYmin) / 2;
  float avg_delta_z = (magZmax - magZmin) / 2;
  
  float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3;
  
  magXscale = avg_delta / avg_delta_x;
  magYscale = avg_delta / avg_delta_y;
  magZscale = avg_delta / avg_delta_z;
  
  kalibrasyon_yapildi = true;
  
  // Kalibrasyon sonuçlarını yazdır
  /*
  Serial.println("Kalibrasyon değerleri:");
  Serial.print("X Offset: "); Serial.print(magXoffset);
  Serial.print(" Scale: "); Serial.println(magXscale);
  Serial.print("Y Offset: "); Serial.print(magYoffset);
  Serial.print(" Scale: "); Serial.println(magYscale);
  Serial.print("Z Offset: "); Serial.print(magZoffset);
  Serial.print(" Scale: "); Serial.println(magZscale);
  */
}

// Yön bilgisini döndüren fonksiyon
String yonBul(float heading) {
  if (heading >= 337.5 || heading < 22.5) return "Kuzey";
  else if (heading >= 22.5 && heading < 67.5) return "Kuzeybatı";
  else if (heading >= 67.5 && heading < 112.5) return "Batı";
  else if (heading >= 112.5 && heading < 157.5) return "Güneybatı";
  else if (heading >= 157.5 && heading < 202.5) return "Güney";
  else if (heading >= 202.5 && heading < 247.5) return "Güneydoğu";
  else if (heading >= 247.5 && heading < 292.5) return "Doğu";
  else return "Kuzeydoğu";
}

void rawXyzYazdir(){
  Serial.print("rawX: "); Serial.println(rawX);
  Serial.print("rawY: "); Serial.println(rawY);
  Serial.print("rawZ: "); Serial.println(rawZ);
  Serial.println();
}

/**************************************
 * OTONOM FONKSİYONLARI
 **************************************/

void belirliYoneGit(int yonNum, int sure){
  
  unsigned long timeout = millis();
  dogruYoneDondu = false;

  float hedefAci;

  if (yonNum == 0) hedefAci = 0;        // Kuzey
  else if (yonNum == 1) hedefAci = 90;  // Batı
  else if (yonNum == 2) hedefAci = 180; // Güney
  else if (yonNum == 3) hedefAci = 270; // Doğu
  else return;

  while (!dogruYoneDondu && millis() - timeout < 10000) {
    float mevcutHeading = pusulaOku();
    float fark = hedefAci - mevcutHeading;
    
    // 180 dereceyi geçerse farkı düzelt
    if (fark > 180) fark -= 360;
    if (fark < -180) fark += 360;

    Serial.print("Heading: "); Serial.print(mevcutHeading);
    Serial.print(" | Fark: "); Serial.println(fark);

    if (abs(fark) < 10) {  // Yön farkı 5 derece oluncaya kadar dönme işlemi
      dur();
      dogruYoneDondu = true;
    } else {
      if (fark > 0)
        solaDon(180);  // Sol dönüş
      else
        sagaDon(180);  // Sağ dönüş
    }

    delay(20); // Pusula okumaları arası delay
  }
  
  // Hedefe ulaşınca ileri git
  Serial.println("Otonom ileri başlıyor");
  delayStart = millis();
  while (millis() - delayStart < sure * 1000) {
    ileri();
    delay(50);
  }
  Serial.println("Otonom ileri bitti");

  dur();
}
/**************************************
 * SONAR SENSOR FONKSİYONLARI
 **************************************/

void dalgaYolla(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

float mesafeHesapla(){
  dalgaYolla();

  sure = pulseIn(echoPin, HIGH, 30000UL);

  // Eğer sure 0 ise, echo alınamadı 
  if (sure == 0) {
    Serial.println("Echo alınamadı! (Objenin uzaklığı çok büyük veya yok?)");
    delay(50);
    return 0.0; // Ölçümü iptal edip sonraki döngüye geç
  }

  uzaklik = (sure * SES_HIZI) / 2.0;

  return uzaklik;
}

void carpmaOnle(){
  mesafeHesapla();
  if(uzaklik < 5){
    dur();
  }
}

void uzaklikYaz(){
  Serial.print("Uzaklık: ");
  Serial.print(uzaklik);
  Serial.println(" cm");
}


/**************************************
 * MOTOR FONKSİYONLARI
 **************************************/

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

void sagaDon(int hiz) {
  // Motor hızlarını ayarla
  analogWrite(renablePin1, hiz); analogWrite(renablePin2, hiz);
  analogWrite(lenablePin1, hiz); analogWrite(lenablePin2, hiz);

  // Sağ taraf motorlar TAM GERİ (fren etkisi)
  digitalWrite(rFront1, LOW); digitalWrite(rFront2, HIGH);
  digitalWrite(rBack1, LOW); digitalWrite(rBack2, HIGH);
  
  // Sol taraf motorlar TAM İLERİ
  digitalWrite(lFront1, HIGH); digitalWrite(lFront2, LOW);
  digitalWrite(lBack1, HIGH); digitalWrite(lBack2, LOW);
}

void solaDon(int hiz) {
  // Motor hızlarını ayarla
  analogWrite(renablePin1, hiz); analogWrite(renablePin2, hiz);
  analogWrite(lenablePin1, hiz); analogWrite(lenablePin2, hiz);

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