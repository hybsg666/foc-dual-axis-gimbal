#include <Arduino.h>
#include "foc_control.h"
#include "foc_serial.h"
#include "foc_can.h"
#include "canopen.h"

// 串口波特率。
// ★ 固定在 115200，和 platformio.ini 的 monitor_speed、VOFA 三边保持一致。
//   之前为了量化测试把这里提到 460800，结果监视器那边没同步，全程乱码，
//   连命令都被打成乱码字节——所以看波形这种活没必要用高波特率，稳比快重要。
#define SERIAL_BAUD 115200

void setup() {
  // ★ 接收缓冲必须先于 begin 设置（ESP32 默认只有 256 字节）。
  //   q 命令带 20 条轨迹时约 170 字符，加上 loop 里 printf 阻塞，
  //   256 字节缓冲有溢出丢字符的风险 → 队列被截断、只动一半就停。
  Serial.setRxBufferSize(2048);
  Serial.begin(SERIAL_BAUD);
  delay(500);

  // ★ CAN 先起、CANopen 自检先跑，再初始化电机。
  //   理由：自检要阻塞 200~300ms，这时候电机如果已经使能，位置环就断供 0.3 秒，
  //   云台会往下掉一点再被拉回来。放在"电机还没上电"之前做最干净。
  foc_can_init();
  co_init();
  co_self_test();

  foc_control_init();
}

void loop() {
  foc_serial_update();
  foc_can_update();    
  foc_control_update();
  // ★ 心跳 / TPDO 定时上报 / 底层故障同步都在这里。
  //   少了这一行，CANopen 就只在"串口命令执行的那一刻"才活着：
  //   心跳不发、TPDO 不上报、底层过流也传不到状态字（第 9 步踩过）。
  //   放在 foc_control_update() 之后，是因为 TPDO 要报最新位置。
  co_update();
  foc_control_print();
  // ★ 不加 delay：DengFOCtext10 的 loop 是全速跑(几千 Hz)的，靠高采样率才压得住
  //   I=1 速度环 / I=200 电流环这些大积分增益。之前这里 delay(1) 把 loop 压到 ~270Hz，
  //   同样的 I 参数每步积分量暴增 → 电流环/速度环离散化失稳 → 疯狂原地震。
  //   去掉 delay 后 loop 频率对齐 DengFOC，纯三环参数才能真正生效。
}
