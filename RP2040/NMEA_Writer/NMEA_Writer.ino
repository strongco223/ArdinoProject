#include <Arduino.h>

// 模擬 GPS 輸出參數
float latitude = 23.123456;     // 北緯
float longitude = 121.123456;   // 東經
float altitude = 100.0;         // 高度 (m)
float speed = 0;              // 速度 (節)
float heading = 0.0;          // 航向

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
    simulatedMillis = millis();//+= stepMillis;

    sendGGAGSA();
    delay(20);
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
  // 計算 checksum（不包含 '$'）
  uint8_t checksum = 0;
  for (size_t i = 1; i < sentence.length(); ++i) {
    checksum ^= sentence[i];
  }

  // 準備完整字串，例如：$GPGGA,...*XX\r\n
  char buffer[128];  // 根據最大 NMEA 長度，確保足夠
  snprintf(buffer, sizeof(buffer), "%s*%02X\r\n", sentence.c_str(), checksum);

  // 一次性送出（效率高，避免分段 print）
  Serial2.write((const uint8_t*)buffer, strlen(buffer));

  // For debug: 印出送出的字串到 USB Serial
  Serial.write((const uint8_t*)buffer, strlen(buffer));
}

// --- 各類 NMEA 訊息 ---

float randomOffset(float range) {
  return ((float)random(-1000, 1000) / 1000.0) * range;
}

void sendGGA() {
  float latOffset = randomOffset(0.00005);  // 約5米
  float lonOffset = randomOffset(0.00005);  // 約5米
  float altOffset = randomOffset(0.5);      // 高度微擾

  String lat = toNMEACoordinate(latitude + latOffset, true);
  String lon = toNMEACoordinate(longitude + lonOffset, false);
  String timeStr = getTimeString();

  String sentence = "$GPGGA," + timeStr + "," + lat + ",N," + lon + ",E,1,08,0.9," + String(altitude + altOffset, 1) + ",M,0.0,M,,";

  sendNMEA(sentence);
}

void sendRMC() {
  float latOffset = randomOffset(0.00005);
  float lonOffset = randomOffset(0.00005);
  float spdOffset = randomOffset(0.2);
  float hdgOffset = randomOffset(2.0);

  String lat = toNMEACoordinate(latitude + latOffset, true);
  String lon = toNMEACoordinate(longitude + lonOffset, false);
  String timeStr = getTimeString();

  String sentence = "$GPRMC," + timeStr + ",A," + lat + ",N," + lon + ",E," +
                    String(speed + spdOffset, 1) + "," + String(heading + hdgOffset, 1) + ",310524,,,A";

  sendNMEA(sentence);
}

void sendGSA() {
  float pdop = 1.8 + randomOffset(0.2);
  float hdop = 1.0 + randomOffset(0.1);
  float vdop = 1.2 + randomOffset(0.1);

  String sentence = "$GPGSA,A,3,04,05,09,12,24,25,26,29,30,,,," +
                    String(pdop, 1) + "," + String(hdop, 1) + "," + String(vdop, 1);

  sendNMEA(sentence);
}



void sendGGAGSA() {
  float latOffset = randomOffset(0.000005);
  float lonOffset = randomOffset(0.000005);
  float spdOffset = randomOffset(0.2);
  float hdgOffset = randomOffset(2.0);

  String lat = toNMEACoordinate(latitude + latOffset, true);
  String lon = toNMEACoordinate(longitude + lonOffset, false);
  String timeStr = getTimeString();

  float pdop = 1.5 + randomOffset(0.2);
  float hdop = 0.5 + randomOffset(0.2);
  float vdop = 1.2 + randomOffset(0.1);


  String sentence_GGA = "$GPGGA," + timeStr + "," + lat + ",N," + lon + ",E,1,08,0.9," + altitude + ",M,0.0,M,,";

  String sentence_GSA = "$GPGSA,A,3,04,05,09,12,24,25,26,29,30,,,," +
                    String(pdop, 1) + "," + String(hdop, 1) + "," + String(vdop, 1);

  String sentence_RMC = "$GPRMC," + timeStr + ",A," + lat + ",N," + lon + ",E," +
                    String(speed + spdOffset, 1) + "," + String(heading + hdgOffset, 1) + ",310524,,,A";

  



  sendNMEA(sentence_GGA);
  delay(20);
  sendNMEA(sentence_GSA);
  delay(20);
  sendNMEA(sentence_RMC);

}