#pragma once

// ตั้งค่าเริ่มต้นของขามอเตอร์สั่น
void vibration_begin();

// สั่งเปิด-ปิด มอเตอร์สั่น
void set_vibration(bool enable);

// ดึงสถานะปัจจุบันของมอเตอร์สั่น
bool get_vibration_state();