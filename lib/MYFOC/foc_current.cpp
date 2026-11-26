#include "foc_current.h"

// ADC 相关宏
// 对应 DengFOC 里的 _ADC_VOLTAGE / _ADC_RESOLUTION / _ADC_CONV
#define ADC_VOLTAGE     3.3f
#define ADC_RESOLUTION  4095.0f
#define ADC_CONV        (ADC_VOLTAGE / ADC_RESOLUTION)

// 采样电阻和放大增益
// 对应 DengFOC 里的 _shunt_resistor 和 amp_gain
#define SHUNT_RESISTOR  0.01f
#define AMP_GAIN        50.0f

// 电压到电流的转换系数
// 对应 DengFOC: volts_to_amps_ratio = 1.0 / _shunt_resistor / amp_gain
#define VOLTS_TO_AMPS   (1.0f / SHUNT_RESISTOR / AMP_GAIN)

FOC_Current::FOC_Current() {
    _offset_a = 0;
    _offset_b = 0;
    _gain_a = -VOLTS_TO_AMPS;   // V3P 硬件，负号
    _gain_b = -VOLTS_TO_AMPS;
    current_a = 0;
    current_b = 0;
}

// ------------------------------------------------------------
// 初始化 + 零偏校准
// 对应 DengFOC 的 CurrSense::init()
// ------------------------------------------------------------
void FOC_Current::init(int pinA, int pinB) {
    _pinA = pinA;
    _pinB = pinB;

    pinMode(_pinA, INPUT);
    pinMode(_pinB, INPUT);

    // 读 1000 次求平均，得到零偏
    calibrateOffsets();

    Serial.println("[foc_current] init done");
}

// ------------------------------------------------------------
// 读取单个 ADC 电压
// 对应 DengFOC 的 readADCVoltageInline()
//
// ★ 2026-11-18：加过采样平均。
//   实测（电流指令 iq=0、电机静止）：单次采样时 iq 读数峰峰值 0.54A、std 0.053A。
//   这个噪声不是控制环产生的 —— 位置模式静止测到的 iq 抖动（0.49A）与它几乎一致，
//   说明力矩纹波全部来自 ADC 白噪声。电流环拿这个带噪反馈去调 PWM，
//   等于把噪声直接调制到相电流上 → 力矩纹波 → 电机高频微振 + 细密嗡嗡声。
//   白噪声取 N 次平均可降 √N 倍，且【不引入相位滞后】，不影响电流环带宽，
//   比在电流环里加低通更安全（低通会吃掉相位裕度）。
// ------------------------------------------------------------
// ★ 2026-11-18：过采样次数是【频率/噪声】的直接权衡，不能随便加。
//   实测：单次采样 iq 噪声 p2p 0.54A；8 次平均后 0.088A（降 6 倍），
//   但控制环频率从 ~800Hz 掉到 ~300Hz，同一套 PID 参数每步积分量变化 → 失配。
//   ESP32 的 analogRead 单次约 250us（要切通道 + 校准），是本 loop 最大开销。
//   实测控制环频率：1 次 ≈ 1.1kHz / 2 次 ≈ 965Hz / 4 次 ≈ 420Hz。
//   iq 噪声（指令给 0、静止）：1 次 0.54A → 2 次 0.23A → 4 次 0.21A → 8 次 0.09A。
//   也就是说 4 次以上收益很小却把频率砍半 —— 频率掉到 400Hz 时同一套 PID 的
//   等效积分量变化太大，实测会让 M1 直接开始大幅摆动（"采样率变了 PID 就失配"）。
//   取 2 次：噪声降 40%，频率仍有 965Hz。
#define ADC_OVERSAMPLE 2

float FOC_Current::readADCVoltage(int pin) {
    uint32_t sum = 0;
    for (int i = 0; i < ADC_OVERSAMPLE; i++) {
        sum += analogRead(pin);
    }
    return (sum / (float)ADC_OVERSAMPLE) * ADC_CONV;
}

// ------------------------------------------------------------
// 零偏校准
// 对应 DengFOC 的 calibrateOffsets()
// 电机静止时读 1000 次求平均
// ------------------------------------------------------------
void FOC_Current::calibrateOffsets() {
    const int rounds = 1000;
    float sum_a = 0;
    float sum_b = 0;

    for (int i = 0; i < rounds; i++) {
        sum_a += readADCVoltage(_pinA);
        sum_b += readADCVoltage(_pinB);
        delay(1);
    }

    _offset_a = sum_a / rounds;
    _offset_b = sum_b / rounds;

    Serial.print("[foc_current] offset A = ");
    Serial.print(_offset_a, 4);
    Serial.print(", offset B = ");
    Serial.println(_offset_b, 4);
}

// ------------------------------------------------------------
// 读取两相电流
// 对应 DengFOC 的 getPhaseCurrents()
// 公式：I = (V - offset) * gain
// ------------------------------------------------------------
void FOC_Current::update() {
    current_a = (readADCVoltage(_pinA) - _offset_a) * _gain_a;
    current_b = (readADCVoltage(_pinB) - _offset_b) * _gain_b;
}