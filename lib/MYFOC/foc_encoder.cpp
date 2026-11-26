#include "foc_encoder.h"
#include "esp_log.h"

#define _2PI 6.28318530718f

// 速度合理性上限 (rad/s)。
// 电机 VEL_LIMIT=30，正常速度不会超过它；远高于此的瞬时速度必是编码器毛刺。
// 用于 update() 里丢弃离谱的角度跳变，避免速度/电流被 phantom 尖峰带飞。
#define VEL_SANITY_MAX 50.0f

FOC_Encoder::FOC_Encoder() {
    _angle_prev = 0;
    _angle_prev_ts = 0;
    _full_rotations = 0;
    _vel_angle_prev = 0;
    _vel_angle_prev_ts = 0;
    _vel_full_rotations = 0;
    _last_angle = 0;
    _angle_offset = 0;
    _prev_comp_d = 0;
    _sanity_reject_cnt = 0;
    _i2c_fail_count = 0;
    _last_recover_ms = 0;
    _last_read_ok = false;
}

// ------------------------------------------------------------
// 初始化 I2C 和 AS5600
// 对应 DengFOC 的 Sensor_init()
// ------------------------------------------------------------
void FOC_Encoder::init(TwoWire* wire, int sda, int scl) {
    _wire = wire;
    _sda = sda;
    _scl = scl;

    // ESP32 的 Wire 库每次 I2C 失败都会自己打一行 [E][Wire.cpp:499] ...，
    // 一坏就刷屏、把遥测数据冲没了。这里关掉它，改由下面的计数+自恢复逻辑统一报告。
    esp_log_level_set("Wire", ESP_LOG_NONE);

    // 初始化 I2C，400kHz（对齐 DengFOCtext10：读一次角度只需 ~50µs，是高采样率控制环的前提）。
    // 读失败时下方会返回上次有效角度 + 总线自恢复，偶发错误不会带飞控制环。
    _wire->begin(_sda, _scl, 400000UL);
    _wire->setTimeout(5);   // I2C 异常时最多阻塞 5ms，避免拖死 FOC 控制环

    delay(500);

    // 读几次，稳定数据
    // 对应 DengFOC: 连续调用 getSensorAngle() 三次
    readSensorAngle();
    delay(1);
    _vel_angle_prev = readSensorAngle();
    _vel_angle_prev_ts = micros();

    delay(1);
    readSensorAngle();
    delay(1);
    _angle_prev = readSensorAngle();
    _angle_prev_ts = micros();

    // 记录上电时的机械角度作为原点，使 getFullAngle() 从 0 开始。
    // 这样 p0 的语义变成“停在上电时的位置”，既符合直觉，
    // 又把保持点从机械角 0（2π 接缝处）挪开，避免接缝处噪声误翻圈数。
    _angle_offset = (float)_full_rotations * _2PI + _angle_prev;

    Serial.println("[foc_encoder] init done");
}

// ------------------------------------------------------------
// 读取 AS5600 原始角度
// 对应 DengFOC 的 getSensorAngle()
// AS5600 地址：0x36
// 角度寄存器：0x0C (高字节), 0x0D (低字节)
// 12 位分辨率，范围 0 ~ 4095
// ------------------------------------------------------------
float FOC_Encoder::readSensorAngle() {
    uint8_t angle_reg = 0x0C;
    uint8_t readArray[2];

    // 对应 DengFOC 里的 beginTransmission / write / endTransmission
    _wire->beginTransmission((uint8_t)0x36);
    _wire->write(angle_reg);
    uint8_t err = _wire->endTransmission(false);
    if (err != 0) {
        // I2C 写失败（NAK/超时）：直接返回上次有效角度，避免产生一个离谱的跳变
        _last_read_ok = false;
        _onI2cError();
        return _last_angle;
    }

    // 对应 DengFOC 里的 requestFrom / read
    // 必须检查实际读到的字节数，否则读到 -1(0xFF) 会让角度瞬间蹦到满量程
    // 注意：两个参数都显式转成 uint8_t，消除 ESP32 Wire 的 requestFrom 重载歧义警告
    uint8_t n = _wire->requestFrom((uint8_t)0x36, (uint8_t)2);
    if (n != 2) {
        _last_read_ok = false;
        _onI2cError();
        return _last_angle;
    }
    _i2c_fail_count = 0;    // 读到完整数据，说明总线是好的
    _last_read_ok = true;
    readArray[0] = _wire->read();
    readArray[1] = _wire->read();

    // 拼接 12 位角度值
    // 高 4 位在 readArray[0] 的低 4 位，低 8 位在 readArray[1]
    // 对应 DengFOC 里那一堆 mask，但这里写得更直白
    uint16_t rawValue = ((readArray[0] & 0x0F) << 8) | readArray[1];

    // 转成 0 ~ 2PI
    _last_angle = (rawValue / 4096.0f) * _2PI;
    return _last_angle;
}

// ------------------------------------------------------------
// 更新编码器数据
// 对应 DengFOC 的 Sensor_update()
// 功能：读取新角度，检测过零点，累计圈数
// ------------------------------------------------------------
void FOC_Encoder::update() {
    float val = readSensorAngle();      // I2C 失败时会返回上次有效角度（d_angle≈0）
    uint32_t now = micros();
    float Ts = (now - _angle_prev_ts) * 1e-6f;
    if (Ts <= 0) Ts = 1e-3f;

    float d_angle = val - _angle_prev;

    // ---- 过零补偿：把 ±2π 的跳变折回成连续的小位移 ----
    // 注意：先补偿，再用“补偿后的真实位移”判断速度，才能同时放过真实过零、剔掉毛刺。
    bool wrapped = false;
    if (d_angle >  (0.8f * _2PI)) { d_angle -= _2PI; wrapped = true; }
    else if (d_angle < -(0.8f * _2PI)) { d_angle += _2PI; wrapped = true; }

    // 补偿后的 d_angle 是真实连续位移；其速度若远超电机极限必是毛刺（VEL_LIMIT=30）
    float inst_speed = fabsf(d_angle) / Ts;
    if (inst_speed > VEL_SANITY_MAX) {
        // ★★★ 这里原来有一个"永久冻结"的死锁，是"转一半停住 → 报故障码 4"的直接成因 ★★★
        //   原实现：只更新 _angle_prev_ts，不更新 _angle_prev，然后 return。
        //   下一拍 d_angle 仍然是「旧参考 → 新读数」那个大值，而 Ts 已经回落到一个
        //   正常的小值，于是 inst_speed 反而更大 → 又丢弃 → 参考点永远追不上读数。
        //   结果：角度读数永久冻结在跳变前那一刻。
        //   位置环看到"角度不动"→ 拼命加力 → 电机被顶到线缆/机械限位真的停住
        //   → 速度≈0 且误差>阈值持续 3 秒 → 报 4。看起来就像"转到一半突然死了"。
        //   修法：允许连续丢 2 拍滤掉真毛刺；连续 3 拍仍超限就说明不是毛刺，
        //   而是参考点失配（I2C 坏值/丢更新），强制把参考点重同步到当前读数。
        _sanity_reject_cnt++;
        _angle_prev_ts = now;
        if (_sanity_reject_cnt >= 3) {
            _angle_prev = val;              // 强制重同步，宁可跳一下也不许冻死
            _prev_comp_d = 0.0f;
            _sanity_reject_cnt = 0;
            Serial.printf("[ENC] resync to %.3f (sanity lock broken)\n", val);
        }
        return;
    }
    _sanity_reject_cnt = 0;

    // 真正跨过 0 点才累计圈数。
    // ★ 消抖：只有当“本次补偿后位移”与“上一拍补偿后位移”同号（方向一致）才计圈。
    //   接缝处单点噪声会先 +2π 再 -2π，符号交替，不会累计；真实旋转方向一致，正常累计。
    if (wrapped) {
        if (_prev_comp_d * d_angle >= 0.0f) {
            _full_rotations += (d_angle > 0) ? 1 : -1;
        }
    }

    _prev_comp_d = d_angle;
    _angle_prev = val;
    _angle_prev_ts = now;
}

// ------------------------------------------------------------
// 获取机械角度
// 对应 DengFOC 的 getMechanicalAngle()
// ------------------------------------------------------------
float FOC_Encoder::getMechanicalAngle() {
    return _angle_prev;
}

// ------------------------------------------------------------
// 获取累计角度
// 对应 DengFOC 的 getAngle()
// ------------------------------------------------------------
float FOC_Encoder::getFullAngle() {
    return (float)_full_rotations * _2PI + _angle_prev - _angle_offset;
}

// ------------------------------------------------------------
// 绝对机械角（无偏置）：装配标定用
// ------------------------------------------------------------
float FOC_Encoder::getAbsAngle() {
    return (float)_full_rotations * _2PI + _angle_prev;
}

// ------------------------------------------------------------
// 设角度偏置（标定云台零点）
// 传 getAbsAngle() 的当前值进来，之后 getFullAngle() 在此处 = 0。
// ------------------------------------------------------------
void FOC_Encoder::setAngleOffset(float offset_abs) {
    _angle_offset = offset_abs;
}

// ------------------------------------------------------------
// 获取速度
// 对应 DengFOC 的 getVelocity()
// 计算方式：(累计角度差) / (时间差)
// ------------------------------------------------------------
float FOC_Encoder::getVelocity() {
    // 计算时间差，单位秒
    float Ts = (_angle_prev_ts - _vel_angle_prev_ts) * 1e-6f;
    if (Ts <= 0) Ts = 1e-3f;   // 防止微溢出

    // 计算速度
    float vel = ((float)(_full_rotations - _vel_full_rotations) * _2PI
                 + (_angle_prev - _vel_angle_prev)) / Ts;

    // ★ 过零累计若偶发漏计/多计一圈（M0 编码器噪声在 0°/360° 边界处读数来回跳，
    //   被误判成"过零"），这一拍的 vel 会瞬间出现 ±(2π/Ts)≈±3000 rad/s 的尖峰，
    //   把速度环带飞 → 电机被猛推、多转一整圈。
    //   真实电机速度不可能超过 VEL_SANITY_MAX(50)，超过必是累计错误，丢弃为 0。
    if (fabsf(vel) > VEL_SANITY_MAX) {
      Serial.printf("[ENC] vel spike %.1f -> 0\n", vel);
      vel = 0.0f;
    }

    // 保存本次数据，供下次计算
    _vel_angle_prev = _angle_prev;
    _vel_full_rotations = _full_rotations;
    _vel_angle_prev_ts = _angle_prev_ts;

    return vel;
}

// ------------------------------------------------------------
// I2C 出错处理：连续失败若干次就做一次总线自恢复
// 为什么需要：Error 263/-1 一旦出现，总线可能被从机拉死，之后每次读都失败，
// 角度就“冻结”在旧值上（电机看着静止、其实闭环已经瞎了）。
// 这里主动恢复，能从这种状态里自己爬起来，不用断电。
// ------------------------------------------------------------
void FOC_Encoder::_onI2cError() {
    if (_i2c_fail_count < 60000) _i2c_fail_count++;

    // 连续失败 ≥3 次就动手（总线卡死时每次读都要等超时，会把 1kHz 控制环直接拖死，
    // 所以越快恢复越好）；距上次恢复 >200ms 才再试，避免真坏时反复 end/begin
    uint32_t now = millis();
    if (_i2c_fail_count >= 3 && (now - _last_recover_ms) > 200) {
        _last_recover_ms = now;
        _i2c_fail_count = 0;
        _busRecover();
    }
}

// 标准 I2C 总线恢复：手动打 9 个 SCL 时钟，把被从机拉低的 SDA 释放，再补一个 STOP
void FOC_Encoder::_busRecover() {
    // 打引脚号：两轴各用一条 I2C，M0 = SDA11/SCL12，M1 = SDA13/SCL14。
    // 不打印引脚时，日志只说"总线坏了"，根本不知道是哪根轴的线松了。
    Serial.printf("[foc_encoder] I2C bus error (SDA=%d SCL=%d) -> recovering bus\n",
                  (int)_sda, (int)_scl);

    _wire->end();

    pinMode(_sda, INPUT_PULLUP);
    pinMode(_scl, OUTPUT);
    digitalWrite(_scl, HIGH);
    delayMicroseconds(5);
    for (int i = 0; i < 9; i++) {
        digitalWrite(_scl, LOW);  delayMicroseconds(5);
        digitalWrite(_scl, HIGH); delayMicroseconds(5);
    }
    // 补一个 STOP：SDA 在 SCL 高电平期间由低变高
    pinMode(_sda, OUTPUT);
    digitalWrite(_sda, LOW);   delayMicroseconds(5);
    digitalWrite(_scl, HIGH);  delayMicroseconds(5);
    digitalWrite(_sda, HIGH);  delayMicroseconds(5);

    _wire->begin(_sda, _scl, 400000UL);
    _wire->setTimeout(5);
    esp_log_level_set("Wire", ESP_LOG_NONE);
}