#include "NMEAGenerator.h"
#include <math.h>  // for atan2, sqrt, etc.

NMEAGenerator::NMEAGenerator(HardwareSerial &serial, unsigned long startTime)
    : serialPort(serial),latitude(0.0), longitude(0.0), altitude(0.0), speed(0.0), heading(0.0) {}

void NMEAGenerator::setPosition(float lat, float lon, float alt) {
    unsigned long now = millis();

    if (hasLastPoint) {
        float distance = haversineDistance(lastLat, lastLon, lat, lon);  // 公尺
        float deltaTime = (now - lastUpdateTime) / 1000.0f;               // 秒
        if (deltaTime > 0.1f) {
            speed = distance / deltaTime * 1.94384f;  // m/s → 節(knots)
            heading = calculateBearing(lastLat, lastLon, lat, lon);
        }
    }

    lastLat = lat;
    lastLon = lon;
    lastUpdateTime = now;
    hasLastPoint = true;

    latitude = lat;
    longitude = lon;
    altitude = alt;
}

float NMEAGenerator::haversineDistance(float lat1, float lon1, float lat2, float lon2) {
    const float R = 6371000.0f;  // 地球半徑 (公尺)
    float dLat = radians(lat2 - lat1);
    float dLon = radians(lon2 - lon1);
    float a = sin(dLat / 2) * sin(dLat / 2) +
              cos(radians(lat1)) * cos(radians(lat2)) *
              sin(dLon / 2) * sin(dLon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

// 計算航向角（回傳角度 0~360）
float NMEAGenerator::calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    float dLon = radians(lon2 - lon1);
    float y = sin(dLon) * cos(radians(lat2));
    float x = cos(radians(lat1)) * sin(radians(lat2)) -
              sin(radians(lat1)) * cos(radians(lat2)) * cos(dLon);
    float bearing = degrees(atan2(y, x));
    return fmod(bearing + 360.0f, 360.0f);  // 保證是 0~360
}


void NMEAGenerator::setSpeedHeading(float spd, float hdg) {
    speed = spd;
    heading = hdg;
}

String NMEAGenerator::getTimeString() {
    unsigned long totalSeconds = millis() / 1000;
    unsigned int hours = 9 + (totalSeconds / 3600);   // 模擬從 09:00 開始
    unsigned int minutes = (totalSeconds % 3600) / 60;
    unsigned int seconds = totalSeconds % 60;
    unsigned int ms = millis() % 1000;

    char buffer[16];
    sprintf(buffer, "%02d%02d%02d.%03d", hours % 24, minutes, seconds, ms);
    return String(buffer);
}

String NMEAGenerator::toNMEACoordinate(float decimal, bool isLatitude) {
    int degrees = int(decimal);
    float minutes = (decimal - degrees) * 60;
    char buff[16];
    if (isLatitude)
        sprintf(buff, "%02d%07.4f", degrees, minutes);
    else
        sprintf(buff, "%03d%07.4f", degrees, minutes);
    return String(buff);
}

char NMEAGenerator::calcChecksum(const String &sentence) {
    char cs = 0;
    for (unsigned int i = 1; i < sentence.length(); i++) {
        cs ^= sentence[i];
    }
    return cs;
}

void NMEAGenerator::sendNMEA(const String &sentence) {
    // 計算 checksum（不含開頭的 $）
    uint8_t checksum = 0;
    for (size_t i = 1; i < sentence.length(); ++i) {
        checksum ^= sentence[i];
    }

    // 準備完整字串：$GPGGA,...*XX\r\n
    char buffer[128];  // 預留足夠空間（NMEA 一般 < 100 bytes）
    snprintf(buffer, sizeof(buffer), "%s*%02X\r\n", sentence.c_str(), checksum);

    // 一次性輸出（避免中斷干擾）
    serialPort.write((const uint8_t*)buffer, strlen(buffer));

    // Debug 輸出到 USB Serial
    Serial.write((const uint8_t*)buffer, strlen(buffer));
}

void NMEAGenerator::sendGGA() {
    String lat = toNMEACoordinate(latitude, true);
    String lon = toNMEACoordinate(longitude, false);
    String timeStr = getTimeString();
    String sentence = "$GPGGA," + timeStr + "," + lat + ",N," + lon + ",E,1,08,0.9," + String(altitude, 1) + ",M,0.0,M,,";
    sendNMEA(sentence);
}

void NMEAGenerator::sendRMC() {
    String lat = toNMEACoordinate(latitude, true);
    String lon = toNMEACoordinate(longitude, false);
    String timeStr = getTimeString();
    String sentence = "$GPRMC," + timeStr + ",A," + lat + ",N," + lon + ",E," + String(speed, 1) + "," + String(heading, 1) + ",310524,,,A";
    sendNMEA(sentence);
}

void NMEAGenerator::sendGGARMC() 
{
    String lat = toNMEACoordinate(latitude, true);
    String lon = toNMEACoordinate(longitude, false);
    String timeStr = getTimeString();
    String sentence_1 = "$GPGGA," + timeStr + "," + lat + ",N," + lon + ",E,1,08,0.9," + String(altitude, 1) + ",M,0.0,M,,";
    sendNMEA(sentence_1);
    delay(30);
    String sentence_2 = "$GPRMC," + timeStr + ",A," + lat + ",N," + lon + ",E," + String(speed, 1) + "," + String(heading, 1) + ",310524,,,A";
    sendNMEA(sentence_2);
    delay(30);

}

void NMEAGenerator::sendGSA() {
    String sentence = "$GPGSA,A,3,04,05,09,12,24,25,26,29,30,,,,1.8,1.0,1.2";
    sendNMEA(sentence);
}