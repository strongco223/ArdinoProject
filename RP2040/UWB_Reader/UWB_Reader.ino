#define MAX_PACKET_LEN 129
uint8_t buffer[MAX_PACKET_LEN];
int bufferIndex = 0;
bool packetStarted = false;

int32_t parseInt24(uint8_t *data);
float parseFloat(uint8_t *data);

int32_t parseInt24(uint8_t *data) {
  int32_t value = (data[0] | (data[1] << 8) | (data[2] << 16));
  if (value & 0x800000) value |= 0xFF000000;
  return value;
}

float parseFloat(uint8_t *data) {
  float f;
  uint8_t *ptr = (uint8_t*)&f;
  ptr[0] = data[0];
  ptr[1] = data[1];
  ptr[2] = data[2];
  ptr[3] = data[3];
  return f;
}



void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(921600);
  Serial.println("Start listening...");
}

void loop() {
  while (Serial1.available()) {
    uint8_t byteReceived = Serial1.read();

    // 檢查是否為封包開頭 0x55 0x01
    if (!packetStarted) {
      if (byteReceived == 0x55 && bufferIndex == 0) {
        buffer[bufferIndex++] = byteReceived;
      } else if (bufferIndex == 1 && byteReceived == 0x01) {
        buffer[bufferIndex++] = byteReceived;
        packetStarted = true;
      } else {
        bufferIndex = 0; // 重來
      }
    } else {
      buffer[bufferIndex++] = byteReceived;

      // 收到完整封包
      if (bufferIndex == MAX_PACKET_LEN) {
        Serial.println("=== Got Packet ===");

        // 逐一解析欄位（範例：前幾個欄位）
        Serial.print("Header: 0x"); Serial.println(buffer[0], HEX);
        Serial.print("Function: 0x"); Serial.println(buffer[1], HEX);
        Serial.print("ID: "); Serial.println(buffer[2]);
        Serial.print("Role: "); Serial.println(buffer[3]);

        int32_t pos_x_raw = parseInt24(&buffer[4]);
        int32_t pos_y_raw = parseInt24(&buffer[7]);
        int32_t pos_z_raw = parseInt24(&buffer[10]);

        Serial.print("Pos.X: "); Serial.println(pos_x_raw / 1000.0, 3);
        Serial.print("Pos.Y: "); Serial.println(pos_y_raw / 1000.0, 3);
        Serial.print("Pos.Z: "); Serial.println(pos_z_raw / 1000.0, 3);

        // 重置
        bufferIndex = 0;
        packetStarted = false;
      }
    }
  }
}