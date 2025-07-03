#include "CH9120.h"
#include "NMEAGenerator.h"

NMEAGenerator nmea(Serial1);  // TX=4, RX=5 預設設定在 setup() 中
unsigned long lastUpdate = 0;
unsigned long stepMillis = 500;
float lat, lon, alt;

void setup() {
    CH9120_init();
    Serial1.setTX(0);
    Serial1.setRX(1);
    Serial1.begin(115200);

  Serial.println("GPS NMEA generator started");
  

     
}

void loop() {
    if (UART_ID1.available() && UART_ID1.read() == 0xAA) {
        UCHAR rx[12]; // 剩下 12 bytes 是 3 個 float
        int i = 0;

        while (i < 12) {
            while (!UART_ID1.available()); // 等下一個 byte 到
            rx[i++] = UART_ID1.read();
        }

        
        memcpy(&lat, &rx[0], 4);
        memcpy(&lon, &rx[4], 4);
        memcpy(&alt, &rx[8], 4);

        char buffer[64];
        sprintf(buffer, "Lat: %.7f, Lon: %.7f, Alt: %.3f\n", lat, lon, alt);
        UART_ID1.write((const uint8_t*)buffer, strlen(buffer));


      nmea.setPosition(lat, lon, alt);
      nmea.setSpeedHeading(0, 0);
      nmea.sendGGARMC();
      //nmea.sendGGA();
      //nmea.sendRMC();
      //nmea.sendGSA();

    }
}