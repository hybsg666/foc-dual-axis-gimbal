#ifndef FOC_CAN_H
#define FOC_CAN_H

#include <Arduino.h>

// ===== CAN 协议（两个电机各一组 ID）=====
// 电机 0：
#define CAN_ID_TARGET   0x100   // 上位机 → 驱动器：目标帧
#define CAN_ID_FEEDBACK 0x101   // 驱动器 → 上位机：位置/速度
#define CAN_ID_FAULT    0x102   // 驱动器 → 上位机：电流/故障码

// 电机 1：
#define CAN_ID_TARGET1   0x110
#define CAN_ID_FEEDBACK1 0x111
#define CAN_ID_FAULT1    0x112

// ===== 引脚 =====
// ESP32-S3 + MCP2515（SPI）
// 注意 ESP32-S3 SPI.begin 参数顺序是 (SCK, MISO, MOSI, CS)
#define CAN_SPI_SCK   21
#define CAN_SPI_MISO  39
#define CAN_SPI_MOSI  38
#define CAN_SPI_CS    40
#define CAN_INT_PIN   41

// 初始化
void foc_can_init();

// 每 loop 调用
void foc_can_update();

// 是否初始化成功
bool foc_can_is_ready();

// ============================================================
// 原始帧接口：给 CANopen 协议栈（lib/CANopen）用
// ============================================================
// 发一帧（id 11 位，len ≤ 8）。返回 true = 已塞进 MCP2515 的发送缓冲
bool foc_can_send(uint32_t id, const uint8_t* data, uint8_t len);

// 收一帧，没有帧返回 false（非阻塞）
bool foc_can_recv(uint32_t* id, uint8_t* data, uint8_t* len);

// 切内部回环模式：自己发的帧自己收回，用于"没有主站硬件也能自检"
bool foc_can_set_loopback(bool on);

// 自定义旧帧协议（0x100/0x110 目标帧 + 10ms 反馈帧）
//   迁到 CANopen 后默认关掉，免得总线上一半是自定义帧一半是 CANopen，看着乱。
//   想看旧行为就把它改成 1。
#define CAN_LEGACY_ENABLE 0

#endif
