#include "vibration.h"
#include <Arduino.h>

// กำหนดขาตามที่ออกแบบไว้
static constexpr int VIB_PWM_PIN = 2;
static constexpr int VIB_DIR_PIN = 15;

// ความเร็วในการเขย่าตะแกรง (0-255)
static constexpr int VIB_SPEED = 70; 

// ตัวแปรเก็บสถานะว่าตอนนี้สั่นอยู่ไหม
static bool is_vibrating = false;

void vibration_begin() {
    pinMode(VIB_PWM_PIN, OUTPUT);
    pinMode(VIB_DIR_PIN, OUTPUT);

    // บังคับทิศทางให้หมุนทางเดียวตลอดกาล
    digitalWrite(VIB_DIR_PIN, HIGH);
    
    // ปิดมอเตอร์ไว้ก่อนตอนเริ่มต้น
    analogWrite(VIB_PWM_PIN, 0);
    is_vibrating = false;
}

void set_vibration(bool enable) {
    is_vibrating = enable;
    if (enable) {
        analogWrite(VIB_PWM_PIN, VIB_SPEED); // สั่งสั่นที่ความแรง 70
    } else {
        analogWrite(VIB_PWM_PIN, 0);         // สั่งหยุด
    }
}

bool get_vibration_state() {
    return is_vibrating;
}