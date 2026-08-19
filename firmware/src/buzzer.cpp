#include "buzzer.h"
#include <Arduino.h>

// กำหนดขา Buzzer ตาม PCB (ขา 19 ของบอร์ด encoder_esp)
#define BUZZER_PIN 19

// ตัวแปรสำหรับคุม Timer ของ Buzzer
static bool is_buzzer_on = false;
static uint32_t buzzer_start_time = 0;
static uint32_t buzzer_duration_ms = 0;

void buzzer_begin() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // ปิดไว้ก่อนตอนเริ่มระบบ
}

void set_buzzer_duration(float duration_sec) {
  if (duration_sec > 0.0f) {
    // สั่งเปิด Buzzer และแปลงวินาทีเป็นมิลลิวินาที
    is_buzzer_on = true;
    buzzer_duration_ms = (uint32_t)(duration_sec * 1000.0f);
    buzzer_start_time = millis();
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    // ถ้าค่าเป็น 0.0 ให้ปิดทันที
    is_buzzer_on = false;
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void handle_buzzer() {
  // เช็คเวลาดับ Buzzer ตลอดเวลาที่มันกำลังทำงานอยู่
  if (is_buzzer_on) {
    if (millis() - buzzer_start_time >= buzzer_duration_ms) {
      // ถ้าเวลาผ่านไปครบกำหนดแล้ว ให้ปิด Buzzer
      is_buzzer_on = false;
      digitalWrite(BUZZER_PIN, LOW);
    }
  }
}