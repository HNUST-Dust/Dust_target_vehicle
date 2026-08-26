#ifndef TASK_CMD_UART_H
#define TASK_CMD_UART_H


void task_cmd_uart_init(void);    /* 启动 UART7 中断接收 */
void task_cmd_uart_loop(void *argument);  /* 帧处理任务 */

#endif