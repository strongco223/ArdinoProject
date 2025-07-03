#define UWB_RX_PIN 1
#define UWB_TX_PIN 0
#define FCU_RX_PIN 5
#define FCU_TX_PIN 4

unsigned long lastReportTime = 0;
unsigned int countGGA = 0;
unsigned int countRMC = 0;
unsigned int countGSA = 0;

String currentSentence = "";
bool sentenceStarted = false;

void setup() {
  Serial.begin(115200);
  while (!Serial); // 等待 USB Serial 就緒

  Serial1.setTX(UWB_TX_PIN);
  Serial1.setRX(UWB_RX_PIN);
  Serial1.begin(921600);

  Serial2.setTX(FCU_TX_PIN);
  Serial2.setRX(FCU_RX_PIN);
  Serial2.begin(115200);

  Serial.println("UWB NMEA forwarder started.");
}

void loop() {
  while (Serial1.available()) {
    uint8_t c = Serial1.read();
    Serial2.write(c);

  }
}
