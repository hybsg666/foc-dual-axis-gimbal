#include "foc_can.h"
#include "foc_pwm.h"        // MOTOR_0 / MOTOR_1 / NUM_MOTORS
#include "foc_control.h"
#include "foc_protection.h"
#include "canopen.h"        // 收到的帧交给 CANopen 协议栈
#include <SPI.h>
#include <mcp2515.h>
#include <string.h>

// MCP2515 对象
// 用独立的 SPI 总线（HSPI），并把 SPI 指针传给库。
// 第三个参数 1000000 = 1MHz，比库默认的 10MHz 稳得多，面包板接线必备。
static SPIClass CAN_SPI(HSPI);
static MCP2515 mcp2515(CAN_SPI_CS, 1000000, &CAN_SPI);

static bool     can_ready      = false;

#if CAN_LEGACY_ENABLE
static uint32_t last_fb_ts     = 0;
#define FB_PERIOD_MS 10   // 反馈帧发送周期
#endif

#define MCP_REG_CANSTAT 0x0E

// 不依赖库的私有类型，直接用底层 SPI 读一个寄存器（诊断用）
static uint8_t can_read_reg(uint8_t reg) {
  CAN_SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CAN_SPI_CS, LOW);
  CAN_SPI.transfer(0x03);          // READ 指令
  CAN_SPI.transfer(reg);
  uint8_t val = CAN_SPI.transfer(0x00);
  digitalWrite(CAN_SPI_CS, HIGH);
  CAN_SPI.endTransaction();
  return val;
}

// ------------------------------------------------------------
// 初始化
// ------------------------------------------------------------
void foc_can_init() {
  CAN_SPI.begin(CAN_SPI_SCK, CAN_SPI_MISO, CAN_SPI_MOSI, CAN_SPI_CS);
  pinMode(CAN_INT_PIN, INPUT);

  MCP2515::ERROR err = MCP2515::ERROR_OK;
  for (int i = 0; i < 3; i++) {
    err = mcp2515.reset();
    if (err == MCP2515::ERROR_OK) break;
    delay(50);
  }

  if (err != MCP2515::ERROR_OK) {
    uint8_t stat = can_read_reg(MCP_REG_CANSTAT);
    Serial.printf("[can] MCP2515 reset FAILED (err=%d), CANSTAT=0x%02X\n", (int)err, stat);
    if (stat == 0xFF) {
      Serial.println("[can] 读到 0xFF -> 芯片无应答：检查 VCC/GND、8M 晶振、SPI 接线、CS 引脚电平");
    } else {
      Serial.println("[can] 芯片有应答但失败：检查晶振频率(MCP_8MHZ/MCP_16MHZ?)与 setBitrate 配置");
    }
    can_ready = false;
    return;
  }

  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    Serial.println("[can] MCP2515 setBitrate FAILED");
    can_ready = false;
    return;
  }

  mcp2515.setNormalMode();

  can_ready = true;
  Serial.println("[can] MCP2515 ready @ 500kbps");
}

// ------------------------------------------------------------
bool foc_can_is_ready() {
  return can_ready;
}

// ------------------------------------------------------------
// 原始帧收发（给 CANopen 用）
// ------------------------------------------------------------
bool foc_can_send(uint32_t id, const uint8_t* data, uint8_t len) {
  if (!can_ready) return false;
  struct can_frame tx;
  memset(&tx, 0, sizeof(tx));
  tx.can_id  = id;
  tx.can_dlc = (len > 8) ? 8 : len;
  if (tx.can_dlc > 0) memcpy(tx.data, data, tx.can_dlc);
  return mcp2515.sendMessage(&tx) == MCP2515::ERROR_OK;
}

bool foc_can_recv(uint32_t* id, uint8_t* data, uint8_t* len) {
  if (!can_ready) return false;
  struct can_frame rx;
  if (mcp2515.readMessage(&rx) != MCP2515::ERROR_OK) return false;
  *id  = (uint32_t)rx.can_id;
  *len = rx.can_dlc;
  memcpy(data, rx.data, 8);
  return true;
}

bool foc_can_set_loopback(bool on) {
  if (!can_ready) return false;
  // autowp 库把通用 setMode() 设成 private，只放出这几个具名模式函数
  MCP2515::ERROR e = on ? mcp2515.setLoopbackMode() : mcp2515.setNormalMode();
  return e == MCP2515::ERROR_OK;
}

#if CAN_LEGACY_ENABLE
// ------------------------------------------------------------
// 处理一帧目标：根据 ID 决定发给哪个电机
// ------------------------------------------------------------
static void handleTargetFrame(int motor, struct can_frame* rx) {
  uint8_t mode_byte = rx->data[0];
  uint8_t enable    = rx->data[1];

  union { float f; uint8_t b[4]; } val;
  val.b[0] = rx->data[2];
  val.b[1] = rx->data[3];
  val.b[2] = rx->data[4];
  val.b[3] = rx->data[5];

  if (enable == 0) {
    // 失能：复用清故障逻辑（简化处理）
    foc_protection_clear(motor);
  }

  if (mode_byte == 0) {
    foc_set_mode(motor, MODE_CURRENT);
    foc_set_target_iq(motor, val.f);
  } else if (mode_byte == 1) {
    foc_set_mode(motor, MODE_VELOCITY);
    foc_set_target_vel(motor, val.f);
  } else if (mode_byte == 2) {
    foc_set_mode(motor, MODE_POSITION);
    foc_set_target_pos(motor, val.f);
  }

  foc_protection_notify_cmd(motor);
}
#endif  // CAN_LEGACY_ENABLE

// ------------------------------------------------------------
// 每 loop 调用：收帧 → 交给 CANopen；再（可选）发旧协议反馈
// ------------------------------------------------------------
void foc_can_update() {
  if (!can_ready) return;

  // ===== 接收：把硬件缓冲排空（原来只读一帧，忙的时候会积压）=====
  uint32_t id = 0;
  uint8_t  d[8];
  uint8_t  dl = 0;
  while (foc_can_recv(&id, d, &dl)) {
    // CANopen 先接手；它不认的帧再给旧协议
    if (co_on_frame(id, d, dl)) continue;

#if CAN_LEGACY_ENABLE
    if (id == CAN_ID_TARGET || id == CAN_ID_TARGET1) {
      struct can_frame rx;
      rx.can_id  = id;
      rx.can_dlc = dl;
      memcpy(rx.data, d, 8);
      handleTargetFrame((id == CAN_ID_TARGET) ? MOTOR_0 : MOTOR_1, &rx);
    }
#endif
  }

#if CAN_LEGACY_ENABLE
  // ===== 定时发送反馈 =====
  uint32_t now = millis();
  if (now - last_fb_ts >= FB_PERIOD_MS) {
    last_fb_ts = now;

    for (int motor = 0; motor < NUM_MOTORS; motor++) {
      // -------- 反馈帧：位置 + 速度 --------
      struct can_frame fb1;
      fb1.can_id  = (motor == MOTOR_0) ? CAN_ID_FEEDBACK : CAN_ID_FEEDBACK1;
      fb1.can_dlc = 8;

      float pos = foc_control_get_full(motor);
      float vel = foc_control_get_velocity(motor);
      memcpy(&fb1.data[0], &pos, 4);
      memcpy(&fb1.data[4], &vel, 4);
      mcp2515.sendMessage(&fb1);

      // -------- 故障帧：电流 + 故障码 --------
      struct can_frame fb2;
      fb2.can_id  = (motor == MOTOR_0) ? CAN_ID_FAULT : CAN_ID_FAULT1;
      fb2.can_dlc = 8;

      float iq = foc_control_get_iq(motor);
      uint32_t fault = (uint32_t)foc_protection_get_fault(motor);
      memcpy(&fb2.data[0], &iq, 4);
      memcpy(&fb2.data[4], &fault, 4);
      mcp2515.sendMessage(&fb2);
    }
  }
#endif
}
