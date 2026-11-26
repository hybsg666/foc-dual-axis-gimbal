import io, sys

P = r"D:/FOC/MyFOCProject/5CanOpen/STM32F103/Core/Src/canopen_master.c"
raw = open(P, "rb").read().decode("gbk")
nl = "\r\n" if "\r\n" in raw else "\n"


def N(s):
    return s.replace("\n", nl)


pairs = []

# ---- 1) 新全局变量 ----
pairs.append((
"""static uint8_t  s_sine_on  = 0;""",
"""static uint8_t  s_dual     = 0;      /* 1 = dual mode: +/- drives BOTH axes */
static int32_t  s_target2  = 0;      /* node 2 target (0.01 deg), used in dual mode */
static uint8_t  s_sine_on  = 0;"""))

# ---- 2) SDO 读改成可指定节点 ----
pairs.append((
"""static uint8_t com_sdo_read(uint16_t idx, uint8_t sub, uint32_t *val)""",
"""static uint8_t com_sdo_read_n(uint8_t node, uint16_t idx, uint8_t sub, uint32_t *val)"""))

pairs.append((
"""    if (!com_tx(0x600u + s_node, req, 8)) {
        com_log("  TX fail: CAN mailbox full (bus wiring ?)\\n");
        return 0;
    }
    if (!com_wait(0x580u + s_node, rsp, COM_SDO_TIMEOUT)) {
        com_log("  no reply from 0x%03X in %u ms\\n", 0x580u + s_node, COM_SDO_TIMEOUT);""",
"""    if (!com_tx(0x600u + node, req, 8)) {
        com_log("  TX fail: CAN mailbox full (bus wiring ?)\\n");
        return 0;
    }
    if (!com_wait(0x580u + node, rsp, COM_SDO_TIMEOUT)) {
        com_log("  no reply from 0x%03X in %u ms\\n", 0x580u + node, COM_SDO_TIMEOUT);"""))

pairs.append((
"""    *val = (uint32_t)rsp[4] | ((uint32_t)rsp[5] << 8)
         | ((uint32_t)rsp[6] << 16) | ((uint32_t)rsp[7] << 24);
    return 1;
}""",
"""    *val = (uint32_t)rsp[4] | ((uint32_t)rsp[5] << 8)
         | ((uint32_t)rsp[6] << 16) | ((uint32_t)rsp[7] << 24);
    return 1;
}

/* 第 11 步：老接口固定操作 s_node；双轴要指定节点，所以加一层包装 */
static uint8_t com_sdo_read(uint16_t idx, uint8_t sub, uint32_t *val)
{
    return com_sdo_read_n(s_node, idx, sub, val);
}"""))

# ---- 3) SDO 写改成可指定节点 ----
pairs.append((
"""static uint8_t com_sdo_write16(uint16_t idx, uint8_t sub, uint16_t v)""",
"""static uint8_t com_sdo_write16_n(uint8_t node, uint16_t idx, uint8_t sub, uint16_t v)"""))

pairs.append((
"""    if (!com_tx(0x600u + s_node, req, 8)) return 0;
    if (!com_wait(0x580u + s_node, rsp, COM_SDO_TIMEOUT)) return 0;
    return (rsp[0] == 0x60) ? 1 : 0;
}""",
"""    if (!com_tx(0x600u + node, req, 8)) return 0;
    if (!com_wait(0x580u + node, rsp, COM_SDO_TIMEOUT)) return 0;
    return (rsp[0] == 0x60) ? 1 : 0;
}

static uint8_t com_sdo_write16(uint16_t idx, uint8_t sub, uint16_t v)
{
    return com_sdo_write16_n(s_node, idx, sub, v);
}"""))

# ---- 4) CiA402 单步改成可指定节点 ----
pairs.append((
"""static void com_cia_step(uint16_t cw)""",
"""static void com_cia_step_n(uint8_t node, uint16_t cw)"""))

pairs.append((
"""    if (!com_sdo_write16(0x6040, 0, cw)) {
        com_log("  write 0x6040 = 0x%04X failed\\n", cw);
        return;
    }
    HAL_Delay(10);
    if (com_sdo_read(0x6041, 0, &sw)) {
        com_log("  0x6040 = 0x%04X  ->  status 0x%04X  %s\\n",
                cw, (unsigned)sw, com_st_name(sw));
    }
}""",
"""    if (!com_sdo_write16_n(node, 0x6040, 0, cw)) {
        com_log("  node %u: write 0x6040 = 0x%04X failed\\n", (unsigned)node, cw);
        return;
    }
    HAL_Delay(10);
    if (com_sdo_read_n(node, 0x6041, 0, &sw)) {
        com_log("  node %u: 0x6040 = 0x%04X  ->  status 0x%04X  %s\\n",
                (unsigned)node, cw, (unsigned)sw, com_st_name(sw));
    }
}

static void com_cia_step(uint16_t cw)
{
    com_cia_step_n(s_node, cw);
}"""))

# ---- 5) RPDO 发送改成可指定节点 ----
pairs.append((
"""static void com_rpdo_pos(uint16_t cw, int32_t pos100)
{
    uint8_t d[6];

    d[0] = (uint8_t)(cw & 0xFF);
    d[1] = (uint8_t)(cw >> 8);
    d[2] = (uint8_t)(pos100 & 0xFF);
    d[3] = (uint8_t)((pos100 >> 8) & 0xFF);
    d[4] = (uint8_t)((pos100 >> 16) & 0xFF);
    d[5] = (uint8_t)((pos100 >> 24) & 0xFF);

    com_tx(0x200u + s_node, d, 6);
}""",
"""static void com_rpdo_pos_to(uint8_t node, uint16_t cw, int32_t pos100)
{
    uint8_t d[6];

    d[0] = (uint8_t)(cw & 0xFF);
    d[1] = (uint8_t)(cw >> 8);
    d[2] = (uint8_t)(pos100 & 0xFF);
    d[3] = (uint8_t)((pos100 >> 8) & 0xFF);
    d[4] = (uint8_t)((pos100 >> 16) & 0xFF);
    d[5] = (uint8_t)((pos100 >> 24) & 0xFF);

    com_tx(0x200u + node, d, 6);
}

static void com_rpdo_pos(uint16_t cw, int32_t pos100)
{
    com_rpdo_pos_to(s_node, cw, pos100);
}"""))

# ---- 6) 帮助信息 ----
pairs.append((
"""    com_log("  1   node 1 (M0 yaw)     2   node 2 (M1 pitch)\\r\\n");""",
"""    com_log("  b   dual: enable BOTH axes, then +/- moves yaw+pitch together\\r\\n");
    com_log("  1   node 1 (M0 yaw)     2   node 2 (M1 pitch)\\r\\n");"""))

# ---- 7) 双轴使能 + 小工具 ----
pairs.append((
"""static void com_cmd_disable(void)""",
"""/* ============================================================
 * 第 11 步：双轴
 *   b  -> 两个节点各自走一遍 06 / 07 / 0F，成功后 +/- 同时驱动两轴
 *   1/2 -> 回到单轴（老行为不变）
 * ============================================================ */
static int32_t com_clamp(int32_t v100)
{
    if (v100 >  17000L) return  17000L;
    if (v100 < -17000L) return -17000L;
    return v100;
}

static void com_log_deg(const char *tag, int32_t v100)
{
    long a = (long)(v100 < 0 ? -v100 : v100);

    com_log("   %s %s%ld.%02ld deg\\r\\n", tag,
            (v100 < 0 ? "-" : ""), a / 100, a % 100);
}

static void com_cmd_enable_both(void)
{
    uint32_t v1 = 0;
    uint32_t v2 = 0;
    uint32_t sw = 0;
    uint8_t  ok = 1;
    uint8_t  n;

    com_log("\\r\\n[DUAL] enable node 1 (yaw) + node 2 (pitch)\\r\\n");

    /* 1) 两个节点都进 Operational，否则 PDO 不生效 */
    com_nmt_start(1);
    com_nmt_start(2);
    HAL_Delay(30);

    /* 2) 一个节点一个节点推三步 */
    for (n = 1; n <= 2; n++) {
        com_cia_step_n(n, 0x0006);
        com_cia_step_n(n, 0x0007);
        com_cia_step_n(n, 0x000F);
        if (!com_sdo_read_n(n, 0x6041, 0, &sw) || ((sw & 0x006Fu) != 0x0027)) ok = 0;
    }
    if (!ok) {
        com_log("  DUAL enable FAILED: see which node did not reach Operation enabled\\r\\n");
        s_dual    = 0;
        s_enabled = 0;
        return;
    }

    /* 3) 目标先锁在各自当前位置，别让上电姿态被拽走 */
    com_sdo_read_n(1, 0x6064, 0, &v1);
    com_sdo_read_n(2, 0x6064, 0, &v2);
    s_target  = (int32_t)v1;
    s_target2 = (int32_t)v2;

    s_enabled = 1;
    s_dual    = 1;
    s_sine_on = 0;

    com_log("  DUAL mode ON:  +/- moves BOTH axes\\r\\n");
    com_log_deg("yaw  ", s_target);
    com_log_deg("pitch", s_target2);
    com_log("  tip: press 1 or 2 to go back to a single axis\\r\\n");
}

static void com_cmd_disable(void)"""))

# ---- 8) jog 支持双轴 ----
pairs.append((
"""    s_target += delta;
    com_rpdo_pos(0x000F, s_target);
    com_log("  target %ld.%02ld deg (PDO 0x%03X, 6 bytes)\\r\\n",
            (long)(s_target / 100),
            (long)(s_target < 0 ? (-s_target % 100) : (s_target % 100)),
            0x200u + s_node);
}""",
"""    if (s_dual) {
        long ay;
        long ap;

        s_target  = com_clamp(s_target  + delta);
        s_target2 = com_clamp(s_target2 + delta);

        /* 两帧连着发：0x201 给 yaw，0x202 给 pitch */
        com_rpdo_pos_to(1, 0x000F, s_target);
        com_rpdo_pos_to(2, 0x000F, s_target2);

        ay = (long)(s_target  < 0 ? -s_target  : s_target);
        ap = (long)(s_target2 < 0 ? -s_target2 : s_target2);
        com_log("  DUAL 0x201+0x202  yaw %s%ld.%02ld  pitch %s%ld.%02ld deg\\r\\n",
                (s_target  < 0 ? "-" : ""), ay / 100, ay % 100,
                (s_target2 < 0 ? "-" : ""), ap / 100, ap % 100);
        return;
    }

    s_target = com_clamp(s_target + delta);
    com_rpdo_pos(0x000F, s_target);
    com_log("  target %ld.%02ld deg (PDO 0x%03X, 6 bytes)\\r\\n",
            (long)(s_target / 100),
            (long)(s_target < 0 ? (-s_target % 100) : (s_target % 100)),
            0x200u + s_node);
}"""))

# ---- 9) 单轴使能时退出双轴模式 ----
pairs.append((
"""        s_enabled = 1;
        if (com_sdo_read(0x6064, 0, &v)) {""",
"""        s_enabled = 1;
        s_dual    = 0;
        if (com_sdo_read(0x6064, 0, &v)) {"""))

# ---- 10) 失能时也退出双轴 ----
pairs.append((
"""    com_cia_step(0x0000);       /* Disable voltage -> Switch on disabled */
    s_enabled = 0;
}""",
"""    com_cia_step(0x0000);       /* Disable voltage -> Switch on disabled */
    s_enabled = 0;
    s_dual    = 0;
}"""))

# ---- 11) 命令分发 ----
pairs.append((
"""    case '1': s_node = 1; s_enabled = 0; s_sine_on = 0;
              com_log("switch to node 1 (M0 yaw)\\r\\n"); break;
    case '2': s_node = 2; s_enabled = 0; s_sine_on = 0;
              com_log("switch to node 2 (M1 pitch)\\r\\n"); break;""",
"""    case 'b': case 'B': com_cmd_enable_both(); break;
    case '1': s_node = 1; s_enabled = 0; s_sine_on = 0; s_dual = 0;
              com_log("switch to node 1 (M0 yaw)\\r\\n"); break;
    case '2': s_node = 2; s_enabled = 0; s_sine_on = 0; s_dual = 0;
              com_log("switch to node 2 (M1 pitch)\\r\\n"); break;"""))

for i, (old, new) in enumerate(pairs, 1):
    o = N(old)
    c = raw.count(o)
    if c != 1:
        print("PATTERN %d matched %d times -> ABORT" % (i, c))
        print(repr(old[:80]))
        sys.exit(1)
    raw = raw.replace(o, N(new))

open(P, "wb").write(raw.encode("gbk"))
print("patched OK, %d substitutions" % len(pairs))
