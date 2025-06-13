#include <Arduino.h>

// 模擬 GPS 輸出參數
float latitude = 23.123456;     // 北緯
float longitude = 121.123456;   // 東經
float altitude = 100.0;         // 高度 (m)
float speed = 5.5;              // 速度 (節)
float heading = 270.0;          // 航向

unsigned long lastUpdate = 0;
unsigned long simulatedMillis = 0; // 模擬時間 (ms)
unsigned long stepMillis = 500;    // 每次遞增 0.5 秒

void setup() {
  Serial.begin(115200);  // USB Serial (debug)
  Serial2.setTX(4);
  Serial2.setRX(5);
  Serial2.begin(115200);  // RX=GPIO5, TX=GPIO4
  Serial.println("NMEA mocker started.");
}

void loop() {
  if (millis() - lastUpdate >= stepMillis) {
    lastUpdate = millis();
    simulatedMillis += stepMillis;

    sendGGA();
    delay(10);
    sendRMC();
    delay(10);
    sendGSA();
  }
}

// --- 時間字串產生器 ---
String getTimeString() {
  unsigned long totalSeconds = simulatedMillis / 1000;
  unsigned int hours = 9 + (totalSeconds / 3600);   // 從 09:00:00 開始
  unsigned int minutes = (totalSeconds % 3600) / 60;
  unsigned int seconds = totalSeconds % 60;
  unsigned int ms = simulatedMillis % 1000;

  char buffer[16];
  sprintf(buffer, "%02d%02d%02d.%03d", hours, minutes, seconds, ms);
  return String(buffer);
}

// --- NMEA 輔助函式 ---

String toNMEACoordinate(float decimal, bool isLatitude) {
  int degrees = int(decimal);
  float minutes = (decimal - degrees) * 60;
  char buff[16];
  if (isLatitude)
    sprintf(buff, "%02d%07.4f", degrees, minutes);
  else
    sprintf(buff, "%03d%07.4f", degrees, minutes);
  return String(buff);
}

char calcChecksum(const String &sentence) {
  char cs = 0;
  for (unsigned int i = 1; i < sentence.length(); i++) {
    cs ^= sentence[i];
  }
  return cs;
}

void sendNMEA(const String &sentence) {
  char checksum = calcChecksum(sentence);
  Serial2.print(sentence);
  Serial2.print("*");
  
  if (checksum < 16) {
    Serial2.print("0");
    Serial.print("0");
  }
  Serial2.print(String(checksum, HEX));
  Serial2.print("\r\n");  // 正確 NMEA 結尾

  Serial.print(sentence);
  Serial.print("*");
  Serial.println(String(checksum, HEX));
}

// --- 各類 NMEA 訊息 ---

void sendGGA() {
  String lat = toNMEACoordinate(latitude, true);
  String lon = toNMEACoordinate(longitude, false);
  String timeStr = getTimeString();
  String sentence = "$GPGGA," + timeStr + "," + lat + ",N," + lon + ",E,1,08,0.9," + String(altitude, 1) + ",M,0.0,M,,";
  sendNMEA(sentence);
}

void sendRMC() {
  String lat = toNMEACoordinate(latitude, true);
  String lon = toNMEACoordinate(longitude, false);
  String timeStr = getTimeString();
  String sentence = "$GPRMC," + timeStr + ",A," + lat + ",N," + lon + ",E," + String(speed, 1) + "," + String(heading, 1) + ",310524,,,A";
  sendNMEA(sentence);
}

void sendGSA() {
  String sentence = "$GPGSA,A,3,04,05,09,12,24,25,26,29,30,,,,1.8,1.0,1.2";
  sendNMEA(sentence);
}