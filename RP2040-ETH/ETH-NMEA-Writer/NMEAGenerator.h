#ifndef NMEA_GENERATOR_H
#define NMEA_GENERATOR_H

#include <Arduino.h>

class NMEAGenerator {
public:
    NMEAGenerator(HardwareSerial &serial, unsigned long startTime = 0);

    void setPosition(float lat, float lon, float alt);
    void setSpeedHeading(float spd, float hdg);
    void tickTime(unsigned long stepMs = 500);

    void sendGGA();
    void sendRMC();
    void sendGSA();
    void sendGGARMC();

private:
    HardwareSerial &serialPort;
    float latitude, longitude, altitude;
    float speed, heading;
    float lastLat, lastLon;
    unsigned long lastUpdateTime;
    bool hasLastPoint = false;

    String getTimeString();
    float haversineDistance(float lat1, float lon1, float lat2, float lon2);
    float calculateBearing(float lat1, float lon1, float lat2, float lon2);
    String toNMEACoordinate(float decimal, bool isLatitude);
    char calcChecksum(const String &sentence);
    void sendNMEA(const String &sentence);
    
};

#endif