# -*- coding: utf-8 -*-
"""根本修复：串口接收改成 中断 + 环形缓冲。
   F103 的 USART1 只有 1 字节接收寄存器（无 FIFO），一次发 6 个字符
   每 87us 来一个，主循环轮询慢一拍就丢字节 -> 表现就是"敲了没反应"。
"""
import os, sys

P = os.path.abspath(os.path.join(os.path.dirname(__file__), '..',
                                 'STM32F103', 'Core', 'Src', 'canopen_master.c'))
t = open(P, 'rb').read().decode('gbk')

def rep(old, new):
    global t
    if t.count(old) != 1:
        print('FAIL anchor count =', t.count(old)); sys.exit(1)
    t = t.replace(old, new)

# 1) com_init 末尾开启中断接收
rep('''    com_log("\\r\\n[CAN] master ready: 500kbps, PA12=TX PA11=RX\\r\\n");
    com_log("[CAN] press h for help\\r\\n");
}''',
'''    /* 串口改成中断接收：F103 只有 1 字节接收寄存器，轮询收多字符必丢 */
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);

    com_log("\\r\\n[CAN] master ready: 500kbps, PA12=TX PA11=RX\\r\\n");
    com_log("[CAN] press h for help\\r\\n");
}''')

# 2) com_init 之前插入环形缓冲和回调
rep('''void com_init(void)
{''',
'''/* ============================================================
 * 串口接收：中断 + 环形缓冲
 *   F103 的 USART1 没有接收 FIFO，一次发 "90 90=" 是 6 字节、
 *   每 87us 到达一个，主循环轮询只要慢一拍就覆盖丢字节。
 * ============================================================ */
static uint8_t          s_rx_byte;
static uint8_t          s_rx_ring[64];
static volatile uint8_t s_rx_head = 0;
static volatile uint8_t s_rx_tail = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t n;

    if (huart->Instance != USART1) return;

    n = (uint8_t)((s_rx_head + 1u) & 63u);
    if (n != s_rx_tail) {              /* 缓冲满就丢弃新字节，不覆盖旧数据 */
        s_rx_ring[s_rx_head] = s_rx_byte;
        s_rx_head = n;
    }
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

/* 溢出(ORE)/帧错误：必须重启接收，否则 HAL 把 RxState 卡在 BUSY_RX 再也不收 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->RxState = HAL_UART_STATE_READY;
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);
}

void com_init(void)
{''')

# 3) com_poll 改成从环形缓冲取
rep('''void com_poll(void)
{
    uint8_t c;


    /* ---- 串口：先看有没有收到字节，有才去读（不用中断，避免和 CubeMX 的 NVIC 打架）---- */
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET) {
        if (HAL_UART_Receive(&huart1, &c, 1, 10) == HAL_OK) {
            com_cmd(c);
        }
    }
''',
'''void com_poll(void)
{
    /* ---- 串口：字符已由中断存进环形缓冲，这里取出来逐字处理 ---- */
    while (s_rx_tail != s_rx_head) {
        uint8_t c = s_rx_ring[s_rx_tail];

        s_rx_tail = (uint8_t)((s_rx_tail + 1u) & 63u);
        com_cmd(c);
    }
''')

open(P, 'wb').write(t.encode('gbk'))
print('patched6 OK')
