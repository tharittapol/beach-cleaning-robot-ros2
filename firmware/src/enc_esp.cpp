#include <Arduino.h>
#include <ArduinoJson.h> 
#include "encoder.h"
#include "buzzer.h"

// ===================== UART2 (เชื่อมต่อกับ main_esp) =====================
static constexpr uint32_t ENC_UART_BAUD = 115200;
static constexpr int ENC_UART_RX_PIN = 16;
static constexpr int ENC_UART_TX_PIN = 17;

// ===================== Timing =====================
static constexpr uint32_t TX_PERIOD_MS = 20;   // ส่งข้อมูลออกที่ 50 Hz
static constexpr uint32_t ENC_UPDATE_MS = 10;  // อัปเดต Encoder ที่ 100 Hz

static uint32_t last_upd_ms = 0;
static uint32_t last_tx_ms  = 0;

void setup() {
  Serial.begin(115200);  // local monitor (USB)
  
  // เริ่มต้น Serial2 สำหรับส่งข้อมูลไป main_esp
  Serial2.begin(ENC_UART_BAUD, SERIAL_8N1, ENC_UART_RX_PIN, ENC_UART_TX_PIN);
  Serial2.setRxBufferSize(256);

  // เริ่มต้น Hardware ต่างๆ ของบอร์ดนี้
  encoder_begin();
  buzzer_begin();

  last_upd_ms = millis();
  last_tx_ms  = millis();
}

static inline float safeFloat(float x) {
  if (!isfinite(x)) return 0.0f;
  if (fabsf(x) < 0.0001f) return 0.0f; 
  return x;
}

void loop() {
  // 1. รับคำสั่ง (เช่น "B:1.5") จาก main_esp
  if (Serial2.available() > 0) {
    String cmd = Serial2.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("B:")) {
      float duration_sec = cmd.substring(2).toFloat();
      set_buzzer_duration(duration_sec);
    }
  }

  // 2. อัปเดตสถานะ Buzzer
  handle_buzzer();

  const uint32_t now = millis();

  // 3. อ่านค่าและคำนวณ Encoder (100Hz)
  if (now - last_upd_ms >= ENC_UPDATE_MS) {
    const float dt_s = (now - last_upd_ms) / 1000.0f;
    last_upd_ms = now;
    encoder_update(dt_s);
  }

  // 4. สร้าง JSON ส่งกลับไปให้ main_esp (50Hz)
  if (now - last_tx_ms >= TX_PERIOD_MS) {
    last_tx_ms = now;
    EncoderData e = encoder_get();

    StaticJsonDocument<256> doc;
    
    JsonArray enc_vel = doc.createNestedArray("enc_vel");
    // ครอบด้วย safeFloat() ให้หมด
    enc_vel.add(safeFloat(e.w[0].vel_mps)); 
    enc_vel.add(safeFloat(e.w[1].vel_mps));
    enc_vel.add(safeFloat(e.w[2].vel_mps)); 
    enc_vel.add(safeFloat(e.w[3].vel_mps));

    JsonArray wheel_cnt = doc.createNestedArray("wheel_cnt");
    // ส่วน count เป็นจำนวนเต็มอยู่แล้ว ไม่ต้องครอบ
    wheel_cnt.add(e.w[0].count); wheel_cnt.add(e.w[1].count);
    wheel_cnt.add(e.w[2].count); wheel_cnt.add(e.w[3].count);

    doc["t_ms"] = now;

    serializeJson(doc, Serial2);
    Serial2.println(); 
  }
}