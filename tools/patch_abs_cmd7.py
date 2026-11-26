# -*- coding: utf-8 -*-
"""把串口中断初始化提到 com_init 最前面：
   万一 CAN 初始化失败提前 return，串口命令也不能跟着失效（那是唯一的调试通道）。
"""
import os, sys

P = os.path.abspath(os.path.join(os.path.dirname(__file__), '..',
                                 'STM32F103', 'Core', 'Src', 'canopen_master.c'))
t = open(P, 'rb').read().decode('gbk')

old = '''    if (HAL_CAN_ConfigFilter(&hcan, &f) != HAL_OK) {'''
new = '''    /* 串口中断最先开：即使下面 CAN 初始化失败，调试通道也得活着 */
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);

    if (HAL_CAN_ConfigFilter(&hcan, &f) != HAL_OK) {'''
if t.count(old) != 1:
    print('FAIL count', t.count(old)); sys.exit(1)
t = t.replace(old, new)

# 删掉原来放在末尾的那一份
old2 = '''    /* 串口改成中断接收：F103 只有 1 字节接收寄存器，轮询收多字符必丢 */
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1);

    com_log("\\r\\n[CAN] master ready'''
new2 = '''    com_log("\\r\\n[CAN] master ready'''
if t.count(old2) != 1:
    print('FAIL count2', t.count(old2)); sys.exit(1)
t = t.replace(old2, new2)

open(P, 'wb').write(t.encode('gbk'))
print('patched7 OK')
