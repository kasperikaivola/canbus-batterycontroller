#include <ESP32-TWAI-CAN.hpp>

// --- Hardware mapping ---
#define CAN_TX 8      // ESP32 GPIO4  -> transceiver TXD
#define CAN_RX 9      // ESP32 GPIO15 -> transceiver RXD
#define LED_PIN 2

// --- Bus settings ---
static constexpr uint32_t CAN_KBIT     = 250;   // 250 kbps
static constexpr uint32_t TX_PERIOD_MS = 2000;  // send keep-alives every 2 s
static constexpr uint32_t TX_TIMEOUT   = 1;     // ms per frame (driver queue)

// Uncomment to dump any received frames
//#define PRINT_RX

CanFrame rxFrame;

static bool send_extd_frame(uint32_t id, const uint8_t* data, uint8_t len) {
  CanFrame f = {0};
  f.identifier       = id;       // 29-bit ID
  f.extd             = 1;        // extended frame
  f.data_length_code = len > 8 ? 8 : len;
  for (uint8_t i = 0; i < f.data_length_code; ++i) f.data[i] = data[i];
  return ESP32Can.writeFrame(f, TX_TIMEOUT);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  pinMode(LED_PIN, OUTPUT);
  // Pins and queues
  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setRxQueueSize(10);
  ESP32Can.setTxQueueSize(10);

  // Start CAN @ 250 kbps
  // (convertSpeed expects kbit/s as an integer, e.g., 250, 500, 1000)
  const TwaiSpeed spd = ESP32Can.convertSpeed(CAN_KBIT);
  if (!ESP32Can.begin(spd, CAN_TX, CAN_RX, 10, 10)) {
    Serial.println("CAN init failed");
    while (true) delay(1000);
  }
  Serial.println("CAN started @ 250 kbps, extd frames enabled by message flag");
}

void loop() {
  static uint32_t last_tx = 0;
  const uint32_t now = millis();

  if (now - last_tx >= TX_PERIOD_MS) {
    last_tx = now;
    digitalWrite (LED_PIN, HIGH);
    delay(100);
    // 1) 03FF1602#04 3B 8F 00 00 2B 04 12
    static const uint8_t m1[8] = {0x04, 0x3B, 0x8F, 0x00, 0x00, 0x2B, 0x04, 0x12};
    if (!send_extd_frame(0x03FF1602, m1, 8)) {
      Serial.println("tx msg1 failed");
    }
    digitalWrite (LED_PIN, LOW);
    // small inter-frame gap (optional but healthy)
    delay(200);
    digitalWrite (LED_PIN, HIGH);
    delay(100);
    // 2) 02FF2602#DD B8 FB 43 3D 70 49 DB
    static const uint8_t m2[8] = {0xDD, 0xB8, 0xFB, 0x43, 0x3D, 0x70, 0x49, 0xDB};
    if (!send_extd_frame(0x02FF2602, m2, 8)) {
      Serial.println("tx msg2 failed");
    }
    digitalWrite (LED_PIN, LOW);
  }

#ifdef PRINT_RX
  // Non-blocking read (timeout = 0 ms)
  if (ESP32Can.readFrame(rxFrame, 0)) {
    Serial.printf("RX %s %08lX DLC=%u  ",
                  rxFrame.extd ? "EXT" : "STD",
                  (unsigned long)rxFrame.identifier,
                  rxFrame.data_length_code);
    for (uint8_t i = 0; i < rxFrame.data_length_code; ++i) {
      Serial.printf("%02X ", rxFrame.data[i]);
    }
    Serial.println();
  }
#endif
}