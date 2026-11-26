#ifndef FOC_CURRENT_H
#define FOC_CURRENT_H

#include <Arduino.h>

// ------------------------------------------------------------
// 电流采样类
// 对应 DengFOC 的 CurrSense 类
// 功能：
//   1. 读两路 ADC 电压
//   2. 上电零偏校准
//   3. 转换成安培
// ------------------------------------------------------------
class FOC_Current {
public:
    FOC_Current();

    // 初始化 + 零偏校准
    // 对应 DengFOC 的 CurrSense::init()
    // 参数：
    //   pinA, pinB : 两相电流采样 ADC 引脚
    // 注意：校准时电机必须静止且不使能
    void init(int pinA, int pinB);

    // 读取两相电流，更新内部 current_a / current_b
    // 对应 DengFOC 的 CurrSense::getPhaseCurrents()
    void update();

    // 获取两相电流（单位：A）
    float getCurrentA() { return current_a; }
    float getCurrentB() { return current_b; }

private:
    // 读取单个 ADC 引脚的电压
    // 对应 DengFOC 的 readADCVoltageInline()
    float readADCVoltage(int pin);

    // 零偏校准
    // 对应 DengFOC 的 calibrateOffsets()
    void calibrateOffsets();

    int _pinA, _pinB;

    // ADC 零偏电压
    float _offset_a, _offset_b;

    // 转换系数（电压 → 电流）
    float _gain_a, _gain_b;

    // 最近一次读到的电流值
    float current_a, current_b;
};

#endif