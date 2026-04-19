#include "usart.h"
#include <stdio.h>
#include <rt_misc.h>

extern UART_HandleTypeDef huart1;   // 想用USART6就改成 huart6

#pragma import(__use_no_semihosting)

struct __FILE { int handle; };
FILE __stdout;

// 必须提供：避免库去找 semihosting 的 _ttywrch
void _ttywrch(int ch)
{
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart1, &c, 1, 10);
}

// 可选但推荐：避免某些库符号依赖
char *_sys_command_string(char *cmd, int len)
{
    (void)cmd;
    (void)len;
    return 0;
}

void _sys_exit(int x)
{
    (void)x;
    while (1) {}
}

int fputc(int ch, FILE *f)
{
    (void)f;
    uint8_t c = (uint8_t)ch;
    HAL_UART_Transmit(&huart1, &c, 1, 10);
    return ch;
}
