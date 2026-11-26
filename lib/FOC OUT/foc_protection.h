#ifndef FOC_PROTECTION_H
#define FOC_PROTECTION_H

#include <Arduino.h>

// ===== 故障码（bit 位标志，可叠加；VOFA 第 5 列显示的就是这个值）=====
//   0 = 无故障
//   1 = 过流     FAULT_OVERCURRENT    |iq| > OC_THRESHOLD
//   2 = 堵转     FAULT_STALL          电流大 + 位移不变持续 STALL_TIME_MS
//   4 = 位置超差 FAULT_POS_ERROR      位置误差 > POS_ERROR_LIMIT 持续（仅位置模式）
//   8 = 指令超时 FAULT_CMD_TIMEOUT    超过 CMD_TIMEOUT_MS 没收到指令
// 多个故障同时发生时按位或叠加，例如 3 = 过流 + 堵转
#define FAULT_NONE        0
#define FAULT_OVERCURRENT (1 << 0)   // = 1  过流
#define FAULT_STALL       (1 << 1)   // = 2  堵转
#define FAULT_POS_ERROR   (1 << 2)   // = 4  位置超差
#define FAULT_CMD_TIMEOUT (1 << 3)   // = 8  指令超时

// ===== 位置超差保护（故障码 4）总开关 =====
//   1 = 启用，0 = 关闭
//   自己做项目、调试阶段默认 0：
//     位置环偶尔追不上（负载、齿槽、电源跌落、加减速跟不上）都会触发 4，
//     它只是"提醒"，不是故障原因，却会打断整个测试流程。
//     真要查"转不到位"，用 [SNAP] 那 20 行数据看，比报警有用得多。
//   等云台装好、要长期跑的时候再改成 1，防止指令把结构顶坏。
#define POS_ERROR_PROT_ENABLE 0

// ===== 保护阈值 =====
#define OC_THRESHOLD      3.5f       // 过流阈值 A。硬兜底，必须 > IQ_LIMIT(2.0A)
#define STALL_CURRENT     0.45f      // 堵转判定电流 A
#define STALL_VELOCITY    0.5f       // 堵转判定速度 rad/s
#define STALL_MIN_MOVE    0.15f      // 堵转位移判据：窗口内位移超过它就算“在动”
#define STALL_TIME_MS     2000       // 堵转持续时间 ms
#define POS_ERROR_LIMIT   2.5f       // 位置误差阈值 rad（≈143°，几乎整圈才报）
#define POS_ERROR_TIME_MS 10000      // 位置超差持续时间 ms
#define CMD_TIMEOUT_MS    100000     // 指令超时时间 ms（= 100 秒）

// 初始化（两个电机的保护状态都清零）
void foc_protection_init();

// 每 loop 调用一次，传入指定电机的当前状态
void foc_protection_update(int motor, float iq, float velocity, float pos_error,
                           float cmd_vel, float pos, int mode);

// 每次收到指令时调用（刷新指令时间戳，防“指令超时”误报）
void foc_protection_notify_cmd(int motor);

// 读取指定电机的故障码
int  foc_protection_get_fault(int motor);

// 指定电机是否处于故障
bool foc_protection_is_fault(int motor);

// 清除指定电机的故障（会重新使能 PWM）
void foc_protection_clear(int motor);

#endif
