#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

#include <Arduino.h>

// 初始化 NVS（两个电机各自独立的命名空间）
void foc_config_init();

// 保存 / 读取指定电机的零电角度
void  foc_config_save_zero_angle(int motor, float angle);
// 返回 NAN 表示没有保存过
float foc_config_load_zero_angle(int motor);

// ★ 零电角度与"算它时的换相方向 dir"绑定，防止 dir 改了还拿旧零位硬跑。
//   dir 变了而零位没重算 = 换相角整体错位 = 通电不出力、全部变铜损发热，
//   而且对齐环节是自洽的，日志看不出异常 —— 所以这里做自动检测。
void foc_config_save_zero_dir(int motor, int dir);
// 返回 0 表示从没存过（老固件 / 刚清过）。调用方应把它当作「零位来源不可信」，
// 而不是「不用校验」—— 拿错零位通电的后果是发热不出力，代价远大于 1 秒对齐。
int  foc_config_load_zero_dir(int motor);

// 保存 / 读取指定电机的“对齐模式”
//   true  = ALWAYS(dev)：每次上电都重新对齐（稳，但有 1 秒"吸一下"）
//   false = STORED(prod)：有存档就直接用，跳过对齐（上电完全静止）
void foc_config_save_align_mode(int motor, bool always_align);
bool foc_config_load_align_mode(int motor);     // 默认 true

// 保存 / 读取指定电机的「位置软限位」（云台角度 rad，0 = 云台零点）
// 作用是防多圈绕线、防撞结构。装配标定后用串口 L 命令写入。
void foc_config_save_pos_limit(int motor, float lo, float hi);
// 返回 false = 从没存过（调用方用默认值）
bool foc_config_load_pos_limit(int motor, float& lo, float& hi);

// ★ 云台零点（装配标定）
//   存的是"云台 0° 那一刻编码器的绝对机械角(abs)"。
//   上电时用它设编码器偏置，于是 getFullAngle() 永远从云台 0° 起算 ——
//   不以"上电位置"为 0，否则每次上电零点都在变，p0 就没意义了。
//   串口命令：'h' 查看 / '0h' 把当前位置标定为云台 0°。
void  foc_config_save_pos_zero(int motor, float abs_mech);
// 返回 NAN = 从没标定过（调用方沿用"上电位置=0"的老行为）
float foc_config_load_pos_zero(int motor);

// 清除指定电机的标定参数（只清零电角度，保留对齐模式和软限位）
void foc_config_clear(int motor);

#endif
