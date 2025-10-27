#include <ESP32-TWAI-CAN.hpp>
// ========== Hardware mapping ==========
// GPIO pins are now defined in platformio.ini build_flags for each board
// CAN_TX, CAN_RX, LED_R_PIN, LED_G_PIN, LED_B_PIN, LED_COMMON_ANODE

// ========== Bus settings ==========
static constexpr uint32_t CAN_KBIT        = 250;    // 250 kbps
static constexpr uint32_t TX_PERIOD_MS    = 2000;   // keep-alive / activation
static constexpr uint32_t STATUS_REQ_MS   = 10000;  // periodic status request
static constexpr uint32_t STATUS_TIMEOUT  = 15000;  // consider "no status" a fault
static constexpr uint32_t TX_TIMEOUT      = 1;      // ms per frame

// ========== LEDC (PWM) settings ==========
static constexpr int LEDC_FREQ_HZ   = 5000;
static constexpr int LEDC_RES_BITS  = 8;       // 0..255
static constexpr uint8_t LED_DIM_R    = 14;
static constexpr uint8_t LED_DIM_G    = 14;
static constexpr uint8_t LED_DIM_B    = 28;
static constexpr int CH_R = 0, CH_G = 1, CH_B = 2;

static inline uint32_t pwmLevel(uint8_t v) {
  if (LED_COMMON_ANODE) return (1u << LEDC_RES_BITS) - 1 - v;
  return v;
}
static void ledWrite(uint8_t r, uint8_t g, uint8_t b) {
  ledcWrite(LED_R_PIN, pwmLevel(r));
  ledcWrite(LED_G_PIN, pwmLevel(g));
  ledcWrite(LED_B_PIN, pwmLevel(b));
}
static void ledOff() { ledWrite(0,0,0); }
static void ledWhite(uint8_t v, uint8_t v2, uint8_t v3) { ledWrite(v,v2,v3); }
static void ledYellow(uint8_t v, uint8_t v2) { ledWrite(v,v2,0); }
static void ledRed(uint8_t v) { ledWrite(v,0,0); }
static void ledGreen(uint8_t v) { ledWrite(0,v,0); }

// ========== CAN helpers ==========
CanFrame rxFrame;

static bool send_extd_frame(uint32_t id, const uint8_t* data, uint8_t len) {
  CanFrame f = {0};
  f.identifier       = id;       // 29-bit ID
  f.extd             = 1;        // extended frame
  f.data_length_code = (len > 8) ? 8 : len;
  for (uint8_t i = 0; i < f.data_length_code; ++i) f.data[i] = data[i];
  return ESP32Can.writeFrame(f, TX_TIMEOUT);
}

// ========== State derived from BMS frames ==========
static bool bms_output1a     = false; // x[0] == 1 on 0x05FF4609  (1A only)
static bool bms_output1000w  = false; // x[0] == 3 on 0x05FF4609  (full power)
static bool fault_overpower  = false; // x[7] != 0 on 0x05FF4000
static bool fault_overcurrent= false; // x[3] != 0 on 0x05FF4001
static uint32_t last_status_ms = 0;
static uint32_t boot_ms = 0;
static bool activation_confirmed = false; // true once we get first successful BMS response
static bool was_activated = false;         // tracks if we were previously activated
static uint32_t activation_success_time = 0; // tracks when activation first succeeded

static void update_led_from_state() {
  const uint32_t now = millis();
  const bool status_stale =
      (last_status_ms == 0) ? false : (now - last_status_ms > STATUS_TIMEOUT);
  const bool fault = fault_overpower || fault_overcurrent;

  // Check if activation status changed
  const bool currently_activated = bms_output1000w;
  if (currently_activated && !was_activated) {
    activation_confirmed = true;
    was_activated = true;
    activation_success_time = now; // Record when activation succeeded
  } else if (!currently_activated && was_activated) {
    was_activated = false;
  }

  // Track last printed status and time to avoid spam
  static uint32_t last_print_ms = 0;
  static int last_printed_state = -1; // -1=none, 0=slow_yellow, 1=fast_yellow, 2=green, 3=fault
  int current_state = 0;
  const char* status_msg = nullptr;

  // Priority: Faults > Activation celebration > Active state > Default
  if (fault) {
    // Fast blinking red for fault state (250ms interval)
    static uint32_t last_fault_flash = 0;
    static bool fault_flash_state = false;
    if (now - last_fault_flash >= 250) {
      last_fault_flash = now;
      fault_flash_state = !fault_flash_state;
      if (fault_flash_state) {
        ledRed(LED_DIM_R);
      } else {
        ledOff();
      }
    }
    current_state = 3;
    status_msg = "Fault";
  } else if (activation_confirmed && (now - activation_success_time < 1000)) {
    // Fast blinking yellow for 1 second after activation (200ms interval)
    static uint32_t last_celebration_flash = 0;
    static bool celebration_flash_state = false;
    if (now - last_celebration_flash >= 200) {
      last_celebration_flash = now;
      celebration_flash_state = !celebration_flash_state;
      if (celebration_flash_state) {
        ledYellow(LED_DIM_R, LED_DIM_G);
      } else {
        ledOff();
      }
    }
    current_state = 1;
    status_msg = "Activation_Success";
  } else if (currently_activated) {
    // Solid green when fully activated
    ledGreen(LED_DIM_G);
    current_state = 2;
    status_msg = "BMS_1000W_Active";
  } else {
    // Slow blinking yellow by default (1 second interval)
    static uint32_t last_slow_flash = 0;
    static bool slow_flash_state = false;
    if (now - last_slow_flash >= 1000) {
      last_slow_flash = now;
      slow_flash_state = !slow_flash_state;
      if (slow_flash_state) {
        ledYellow(LED_DIM_R, LED_DIM_G);
      } else {
        ledOff();
      }
    }
    current_state = 0;
    status_msg = "BMS_1A_Standby";
  }

  // Print status only if state changed or 1 second has passed
  if (status_msg != nullptr && (current_state != last_printed_state || now - last_print_ms >= 1000)) {
    Serial.println(status_msg);
    last_print_ms = now;
    last_printed_state = current_state;
  }
}

void setup() {
  Serial.begin(115200);
  //while (!Serial) {}

  // Initialize LED PWM channels (ESP32 3.x API)
  ledcAttach(LED_R_PIN, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttach(LED_G_PIN, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttach(LED_B_PIN, LEDC_FREQ_HZ, LEDC_RES_BITS);

  // Slow blinking yellow LED during setup (1 second interval)
  Serial.println("Booting... LED blinking yellow");
  const uint32_t setup_start = millis();
  while (millis() - setup_start < 3000) {
    // Calculate time in cycle (1000ms on, 1000ms off)
    uint32_t cycle_time = (millis() - setup_start) % 2000;
    if (cycle_time < 1000) {
      ledYellow(LED_DIM_R, LED_DIM_G);
    } else {
      ledOff();
    }
    delay(50); // Small delay to prevent busy-waiting
  }

  // CAN init (pins & queues)
  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setRxQueueSize(20);
  ESP32Can.setTxQueueSize(20);

  const TwaiSpeed spd = ESP32Can.convertSpeed(CAN_KBIT);
  if (!ESP32Can.begin(spd, CAN_TX, CAN_RX, 20, 20)) {
    // Hard fault -> blinking red
    Serial.println("CAN init failed");
    while (true) {
      ledRed(LED_DIM_R);
      delay(250);
      ledOff();
      delay(250);
    }
  }
  Serial.println("CAN started @ 250 kbps (extd via per-frame flag)");
  boot_ms = millis();

  // Show slow blinking yellow for a couple more seconds after boot
  Serial.println("Boot complete. Starting activation...");
  delay(2000);
}

void loop() {
  static uint32_t last_tx       = 0;
  static uint32_t last_statusrq = 0;

  const uint32_t now = millis();

  // --- Keep sending your two activation/keep-alive frames (every 2 s) ---
  if (now - last_tx >= TX_PERIOD_MS) {
    last_tx = now;

    // 1) 03FF1602#04 3B 8F 00 00 2B 04 12  (as in your working sketch)
    static const uint8_t m1[8] = {0x04, 0x3B, 0x8F, 0x00, 0x00, 0x2B, 0x04, 0x12};
    (void)send_extd_frame(0x03FF1602, m1, 8);

    delay(200); // small inter-frame gap

    // 2) 02FF2602#DD B8 FB 43 3D 70 49 DB
    static const uint8_t m2[8] = {0xDD, 0xB8, 0xFB, 0x43, 0x3D, 0x70, 0x49, 0xDB};
    (void)send_extd_frame(0x02FF2602, m2, 8);
  }

  // --- Periodic BMS status request like in YAML (every 10 s) ---
  if (now - last_statusrq >= STATUS_REQ_MS) {
    last_statusrq = now;
    static const uint8_t req[8] = {0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    (void)send_extd_frame(0x03FF1605, req, 8); // prompts BMS to keep sending status
  }

  // --- Non-blocking RX parse to derive bms_output1a / 1000w and faults ---
  while (ESP32Can.readFrame(rxFrame, 0)) {
    if (!rxFrame.extd) continue; // we only care about extended frames

    const uint32_t id = rxFrame.identifier;
    const uint8_t *x = rxFrame.data;

    // 0x05FF4609: level + mode (byte 0)
    if (id == 0x05FF4609 && rxFrame.data_length_code >= 4) {
      const uint8_t mode = x[0];
      bms_output1a    = (mode == 1); // 1A only
      bms_output1000w = (mode == 3); // full output
      last_status_ms  = now;         // fresh status received
    }
    // 0x05FF4610 also indicates life; use it to refresh the status timer
    else if (id == 0x05FF4610 && rxFrame.data_length_code >= 8) {
      last_status_ms = now;
    }
    // Fault sources
    else if (id == 0x05FF4000 && rxFrame.data_length_code >= 8) {
      fault_overpower = (x[7] != 0);
      last_status_ms  = now;
    }
    else if (id == 0x05FF4001 && rxFrame.data_length_code >= 4) {
      fault_overcurrent = (x[3] != 0);
      last_status_ms    = now;
    }

#ifdef PRINT_RX
    Serial.printf("RX EXT %08lX DLC=%u  ", (unsigned long)id, rxFrame.data_length_code);
    for (uint8_t i = 0; i < rxFrame.data_length_code; ++i) Serial.printf("%02X ", x[i]);
    Serial.println();
#endif
  }

  // --- LED: fault (red) > 1A (yellow) > 1000W (white) > off ---
  update_led_from_state();
}
