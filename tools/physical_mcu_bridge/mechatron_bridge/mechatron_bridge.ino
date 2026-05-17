// Mechatron Physical MCU Bridge (ATmega328P / Arduino Uno)
// Receives IO instructions over serial and reports pin inputs back to host.
// This firmware is a stable "runtime" – user sketches are not flashed here.
// Host streams instructions (digital/PWM, requests inputs) and the device executes them.

#include <Arduino.h>

static const uint8_t kVersion = 1;

enum MsgType : uint8_t {
  Hello = 1,
  HelloAck = 2,
  SetPinMode = 10,
  DigitalWriteMsg = 11,
  PwmWrite = 12,
  SetDigitalMask = 13,
  SetPwmBulk = 14,
  RequestInputs = 15,
  InputsReport = 30,
  LogLine = 31,
  ErrorMsg = 32,
};

static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t seed = 0xFFFF) {
  uint16_t crc = seed;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static void write_u16_le(uint16_t v) {
  Serial.write((uint8_t)(v & 0xFF));
  Serial.write((uint8_t)((v >> 8) & 0xFF));
}

static void send_frame(uint8_t type, const uint8_t* payload, uint16_t len) {
  uint8_t header[4 + 1 + 1 + 1 + 2 + 2];
  header[0] = 'M'; header[1] = 'C'; header[2] = 'H'; header[3] = 'T';
  header[4] = kVersion;
  header[5] = type;
  header[6] = 0; // flags
  header[7] = (uint8_t)(len & 0xFF);
  header[8] = (uint8_t)((len >> 8) & 0xFF);
  header[9] = 0;
  header[10] = 0;

  // CRC over version..len + payload, with crc bytes zeroed.
  uint16_t crc = 0;
  crc = crc16_ccitt(header + 4, 1 + 1 + 1 + 2 + 2, 0xFFFF);
  if (len && payload) {
    crc = crc16_ccitt(payload, len, crc);
  }
  header[9] = (uint8_t)(crc & 0xFF);
  header[10] = (uint8_t)((crc >> 8) & 0xFF);

  Serial.write(header, sizeof(header));
  if (len && payload) Serial.write(payload, len);
}

static void send_log(const char* s) {
  send_frame(LogLine, (const uint8_t*)s, (uint16_t)strlen(s));
}

static void send_error(const char* s) {
  send_frame(ErrorMsg, (const uint8_t*)s, (uint16_t)strlen(s));
}

// Current input request masks (host-controlled)
static uint32_t g_digital_mask = 0x000FFFFF;
static uint8_t g_analog_mask = 0x3F;
static uint32_t g_output_mask = 0;
static uint32_t g_pwm_mask = 0;

// PWM cache (Arduino pins 0..19)
static uint8_t g_pwm[20];

static inline bool is_pwm_pin(uint8_t pin) {
  return (pin == 3 || pin == 5 || pin == 6 || pin == 9 || pin == 10 || pin == 11);
}

static uint32_t read_u32_le(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void write_u32_to_payload(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void apply_pwm_bulk(uint32_t pwm_mask, const uint8_t* duty20) {
  g_pwm_mask = pwm_mask & 0x000FFFFF;
  for (uint8_t pin = 0; pin < 20; ++pin) {
    uint8_t d = duty20[pin];
    g_pwm[pin] = d;
    if (is_pwm_pin(pin) && (g_pwm_mask & (1UL << pin))) {
      pinMode(pin, OUTPUT);
      analogWrite(pin, d);
    }
  }
}

static void apply_digital_outputs(uint32_t output_mask, uint32_t bits) {
  g_output_mask = output_mask & 0x000FFFFF;
  bits &= g_output_mask;

  // D0..D13 only (never touch RX/TX so serial remains alive)
  for (uint8_t pin = 0; pin < 14; ++pin) {
    if (pin == 0 || pin == 1) continue; // keep serial alive
    if ((g_output_mask & (1UL << pin)) == 0) continue;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, (bits & (1UL << pin)) ? HIGH : LOW);
  }
  // A0..A5 as digital 14..19 (optional)
  for (uint8_t a = 0; a < 6; ++a) {
    uint8_t pin = 14 + a;
    if ((g_output_mask & (1UL << pin)) == 0) continue;
    uint8_t hw_pin = A0 + a;
    pinMode(hw_pin, OUTPUT);
    digitalWrite(hw_pin, (bits & (1UL << pin)) ? HIGH : LOW);
  }
}

static void report_inputs() {
  uint32_t digital_bits = 0;
  for (uint8_t pin = 0; pin < 20; ++pin) {
    if ((g_digital_mask & (1UL << pin)) == 0) continue;
    if ((g_output_mask & (1UL << pin)) != 0) continue;
    uint8_t read_pin = pin;
    if (pin >= 14) {
      read_pin = (uint8_t)(A0 + (pin - 14));
    }
    // Ensure input mode for reads (host may override with outputs via commands)
    pinMode(read_pin, INPUT);
    if (digitalRead(read_pin) == HIGH) digital_bits |= (1UL << pin);
  }

  uint16_t analog[6] = {0,0,0,0,0,0};
  for (uint8_t a = 0; a < 6; ++a) {
    if ((g_analog_mask & (1u << a)) == 0) continue;
    analog[a] = (uint16_t)analogRead(A0 + a);
  }

  uint8_t payload[4 + 12];
  write_u32_to_payload(payload, digital_bits);
  for (uint8_t i = 0; i < 6; ++i) {
    payload[4 + i * 2 + 0] = (uint8_t)(analog[i] & 0xFF);
    payload[4 + i * 2 + 1] = (uint8_t)((analog[i] >> 8) & 0xFF);
  }

  send_frame(InputsReport, payload, sizeof(payload));
}

// Simple RX buffer for framed parsing
static uint8_t rx[256];
static uint16_t rx_len = 0;

static bool try_parse_one() {
  // Find magic
  uint16_t i = 0;
  while (i + 4 <= rx_len) {
    if (rx[i] == 'M' && rx[i+1] == 'C' && rx[i+2] == 'H' && rx[i+3] == 'T') break;
    i++;
  }
  if (i > 0) {
    memmove(rx, rx + i, rx_len - i);
    rx_len -= i;
  }
  const uint16_t min_sz = 4 + 1 + 1 + 1 + 2 + 2;
  if (rx_len < min_sz) return false;
  if (rx[4] != kVersion) { memmove(rx, rx + 1, rx_len - 1); rx_len -= 1; return true; }

  uint8_t type = rx[5];
  uint16_t len = (uint16_t)rx[7] | ((uint16_t)rx[8] << 8);
  uint16_t crc = (uint16_t)rx[9] | ((uint16_t)rx[10] << 8);
  uint16_t frame_sz = (uint16_t)(min_sz + len);
  if (rx_len < frame_sz) return false;

  // Validate CRC (zero crc bytes)
  uint8_t temp[256];
  if (frame_sz > sizeof(temp)) { send_error("frame too large"); rx_len = 0; return false; }
  memcpy(temp, rx + 4, frame_sz - 4);
  temp[1 + 1 + 1 + 2 + 0] = 0;
  temp[1 + 1 + 1 + 2 + 1] = 0;
  uint16_t calc = crc16_ccitt(temp, frame_sz - 4, 0xFFFF);
  if (calc != crc) {
    // resync by dropping 1 byte
    memmove(rx, rx + 1, rx_len - 1);
    rx_len -= 1;
    return true;
  }

  const uint8_t* payload = rx + min_sz;

  // Handle message
  if (type == Hello) {
    // Reply with HelloAck (board string + caps)
    const char* board = "atmega328p_uno";
    uint8_t out[1 + 32 + 4];
    uint8_t n = (uint8_t)strlen(board);
    out[0] = n;
    memcpy(out + 1, board, n);
    uint32_t caps = 0;
    out[1 + n + 0] = (uint8_t)(caps & 0xFF);
    out[1 + n + 1] = (uint8_t)((caps >> 8) & 0xFF);
    out[1 + n + 2] = (uint8_t)((caps >> 16) & 0xFF);
    out[1 + n + 3] = (uint8_t)((caps >> 24) & 0xFF);
    send_frame(HelloAck, out, (uint16_t)(1 + n + 4));
  } else if (type == SetDigitalMask) {
    if (len >= 8) {
      uint32_t output_mask = read_u32_le(payload);
      uint32_t bits = read_u32_le(payload + 4);
      apply_digital_outputs(output_mask, bits);
    }
  } else if (type == SetPwmBulk) {
    if (len >= 24) {
      uint32_t pwm_mask = read_u32_le(payload);
      apply_pwm_bulk(pwm_mask, payload + 4);
    }
  } else if (type == RequestInputs) {
    if (len >= 5) {
      g_digital_mask = read_u32_le(payload) & 0x000FFFFF;
      g_analog_mask = payload[4] & 0x3F;
      report_inputs();
    }
  } else {
    // Ignore unknown for forward compatibility
  }

  // Pop frame
  memmove(rx, rx + frame_sz, rx_len - frame_sz);
  rx_len -= frame_sz;
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  for (uint8_t i = 0; i < 20; ++i) g_pwm[i] = 0;
  send_log("mechatron_bridge ready");
}

void loop() {
  // Read serial into buffer
  while (Serial.available() > 0 && rx_len < sizeof(rx)) {
    rx[rx_len++] = (uint8_t)Serial.read();
  }
  // Parse as many frames as possible
  while (try_parse_one()) {;}
  // Periodic input report keeps host synced even without explicit polling
  static uint32_t last_report_ms = 0;
  uint32_t now = millis();
  if (now - last_report_ms >= 20) {
    last_report_ms = now;
    report_inputs();
  }
}
