/**
  ******************************************************************************
  * @file    canopen_master.h
  * @brief   F103 当 CANopen 主站：给 ESP32 那块从站发帧、读回数据。
  *
  * 用法（main.c 里两行）：
  *     com_init();          // 在 MX_CAN_Init() 之后调一次
  *     while (1) { com_poll(); }   // 主循环里反复调
  *
  * 串口命令（波特率 115200，插蓝pill 的 USB 就能看）：
  *     h   帮助
  *     t   自检：SDO 读 0x1000 设备类型（验证总线通不通）
  *     r   读当前位置 0x6064
  *     e   使能：NMT + 0x6040 走 06 -> 07 -> 0F 三档
  *     x   失能：0x6040 写 0x0000
  *     +   目标位置 +10°（用 PDO 发，不走 SDO）
  *     -   目标位置 -10°
  *     s   连续正弦跟随开关（±15°，周期 2 秒，跑 20 秒）
  *     1   切到节点 1（M0 yaw）
  *     2   切到节点 2（M1 pitch）
  ******************************************************************************
  */

#ifndef CANOPEN_MASTER_H
#define CANOPEN_MASTER_H

#include "main.h"

/* 启动 CAN：配过滤器（全收）→ HAL_CAN_Start() */
void com_init(void);

/* 主循环里反复调：收串口命令、跑连续轨迹 */
void com_poll(void);

#endif /* CANOPEN_MASTER_H */
