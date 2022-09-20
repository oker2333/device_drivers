#ifndef __UART_TTYS_H
#define __UART_TTYS_H

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <stdint.h>

int uart_init(void);
int uart_write(uint8_t *w_buff, uint16_t len);
int uart_read(uint8_t *buff,uint16_t len);
void uart_close(void);

#endif

