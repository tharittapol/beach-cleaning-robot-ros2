#ifndef BUZZER_H
#define BUZZER_H

// ตั้งค่าเริ่มต้นของขา Buzzer
void buzzer_begin();

// รับค่าจาก Serial แล้วสั่งเปิด Buzzer พร้อมตั้งเวลา
void set_buzzer_duration(float duration_sec);

// ฟังก์ชันสำหรับเรียกใน loop() เพื่อเช็คเวลาดับ Buzzer (Non-blocking)
void handle_buzzer();

#endif