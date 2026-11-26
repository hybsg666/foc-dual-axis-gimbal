#ifndef FOC_ENCODER_H
#define FOC_ENCODER_H

#include <Arduino.h>
#include <Wire.h>

// ------------------------------------------------------------
// 手写 AS5600 编码器类
// 对应 DengFOC 的 Sensor_AS5600 类
// 功能：
//   1. 读取 12 位机械角度 (0 ~ 2PI)
//   2. 累计多圈角度
//   3. 计算速度
// ------------------------------------------------------------
class FOC_Encoder {
public:
    FOC_Encoder();

    // 初始化 I2C 和传感器
    // 对应 DengFOC 的 Sensor_init(TwoWire* _wire)
    void init(TwoWire* wire, int sda, int scl);

    // 更新传感器数据，必须在 loop 里高频调用
    // 对应 DengFOC 的 Sensor_update()
    void update();

    // 获取机械角度 (0 ~ 2PI)
    // 对应 DengFOC 的 getMechanicalAngle()
    float getMechanicalAngle();

    // 获取累计角度 (多圈，连续值)
    // 对应 DengFOC 的 getAngle()
    float getFullAngle();

    // 获取速度 (rad/s)
    // 对应 DengFOC 的 getVelocity()
    float getVelocity();

    // ★ 编码器绝对机械角 = 累计圈数*2π + 单圈角（不做任何偏置）
    //   用途：装配标定时把"云台 0°"所在的位置存进 NVS。
    float getAbsAngle();

    // ★ 设置角度偏置，使 getFullAngle() 从 offset_abs 处开始算 0。
    //   传入的必须是 getAbsAngle() 的值，不是相对量。
    //   标定完零点后调用一次，之后 p0 就永久等于那个物理位置。
    void setAngleOffset(float offset_abs);

    // 上一次 I2C 读取是否成功
    // 用途：对齐/标定前必须确认编码器真的在应答，否则会把“冻结的旧角度”当成结果存进 NVS
    bool isHealthy() { return _last_read_ok; }

private:
    // 读取 AS5600 原始机械角度
    // 对应 DengFOC 的 getSensorAngle()
    float readSensorAngle();

    // I2C 出错计数 + 总线自恢复
    // 现象：Wire 报 requestFrom Error 263 / -1，读不到角度（角度冻结）。
    // 处理：连续失败若干次后，手动打 9 个 SCL 时钟把被拉死的 SDA 释放，再重新 begin()。
    void _onI2cError();
    void _busRecover();

    TwoWire* _wire;
    int _sda, _scl;

    uint16_t _i2c_fail_count;   // 连续失败次数
    uint32_t _last_recover_ms;  // 上次恢复的时间，避免死总线时反复恢复
    bool     _last_read_ok;     // 上一次读取是否成功

    float _angle_prev;          // 上一次机械角度
    uint32_t _angle_prev_ts;    // 上一次更新时间戳
    int32_t _full_rotations;    // 累计圈数

    float _vel_angle_prev;      // 上一次计算速度时的角度
    uint32_t _vel_angle_prev_ts;// 上一次计算速度的时间戳
    int32_t _vel_full_rotations;// 上一次计算速度时的圈数

    float _last_angle;          // 上次有效的机械角度（I2C 读失败时回退用）
    float _angle_offset;        // 上电时的机械角度，使 getFullAngle() 从 0 开始（p0 = 回到上电位置）
    float _prev_comp_d;         // 上一拍补偿后的位移，用于过零计圈的消抖
    uint16_t _sanity_reject_cnt; // 连续因"速度超限"被丢弃的拍数（防止参考点永久冻结）
};

#endif