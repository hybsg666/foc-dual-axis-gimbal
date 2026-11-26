import sys

P = r"D:/FOC/MyFOCProject/5CanOpen/STM32F103/Core/Src/canopen_master.c"
raw = open(P, "rb").read().decode("gbk")
nl = "\r\n" if "\r\n" in raw else "\n"


def N(s):
    return s.replace("\n", nl)


pairs = []

# 1) 故障复位小工具
pairs.append((
"""static void com_cia_step(uint16_t cw)
{
    com_cia_step_n(s_node, cw);
}""",
"""static void com_cia_step(uint16_t cw)
{
    com_cia_step_n(s_node, cw);
}

/* ============================================================
 * 故障复位（第 12 步补）
 *   CiA 402 最硬的一条：Fault 是锁存态，只有 0x6040 的 bit7 上升沿能出来。
 *   所以必须"先让 bit7 归零（发 0x0000），再发 0x0080"，这才有沿。
 *   直接发 0x0080（前面正好也是 0x0080）是没有沿的，什么都不会发生；
 *   发 0x0000 也不过是在 Fault 里被忽略 —— 这两点踩过坑。
 * ============================================================ */
static void com_fault_reset(uint8_t node)
{
    uint32_t sw = 0;

    if (com_sdo_read_n(node, 0x6041, 0, &sw) && ((sw & 0x006Fu) == 0x0008))
        com_log("  node %u was in FAULT, resetting ...\\r\\n", (unsigned)node);

    com_sdo_write16_n(node, 0x6040, 0, 0x0000);   /* bit7 = 0 */
    HAL_Delay(10);
    com_sdo_write16_n(node, 0x6040, 0, 0x0080);   /* bit7 上升沿 -> 复位 */
    HAL_Delay(10);

    if (com_sdo_read_n(node, 0x6041, 0, &sw))
        com_log("  node %u status 0x%04X  %s\\r\\n",
                (unsigned)node, (unsigned)sw, com_st_name(sw));
}"""))

# 2) 单轴使能前先复位故障
pairs.append((
"""    /* 2) 三步推档。不能一步到 0x0F —— 从站会拒绝非法跳转 */
    com_cia_step(0x0006);""",
"""    /* 2) 先复位故障：Fault 里只认 bit7 上升沿，直接发 06 是出不来的 */
    com_fault_reset(s_node);

    /* 3) 三步推档。不能一步到 0x0F —— 从站会拒绝非法跳转 */
    com_cia_step(0x0006);"""))

# 3) 双轴使能：每个节点先复位故障
pairs.append((
"""    /* 2) 一个节点一个节点推三步 */
    for (n = 1; n <= 2; n++) {
        com_cia_step_n(n, 0x0006);""",
"""    /* 2) 一个节点一个节点：先复位故障，再推三步 */
    for (n = 1; n <= 2; n++) {
        com_fault_reset(n);
        com_cia_step_n(n, 0x0006);"""))

# 4) 新增 f 键：手动故障复位
pairs.append((
"""    case 'b': case 'B': com_cmd_enable_both(); break;""",
"""    case 'f': case 'F':
              com_log("\\r\\n[RESET] node %u fault reset\\r\\n", s_node);
              com_fault_reset(s_node);
              break;
    case 'b': case 'B': com_cmd_enable_both(); break;"""))

# 5) 帮助里加 f
pairs.append((
"""    com_log("  b   dual: enable BOTH axes, then +/- moves yaw+pitch together\\r\\n");""",
"""    com_log("  f   fault reset (0x6040 bit7 rising edge), then press e\\r\\n");
    com_log("  b   dual: enable BOTH axes, then +/- moves yaw+pitch together\\r\\n");"""))

for i, (old, new) in enumerate(pairs, 1):
    o = N(old)
    c = raw.count(o)
    if c != 1:
        print("PATTERN %d matched %d -> ABORT" % (i, c))
        print(repr(old[:70]))
        sys.exit(1)
    raw = raw.replace(o, N(new))

open(P, "wb").write(raw.encode("gbk"))
print("patched OK, %d substitutions" % len(pairs))
