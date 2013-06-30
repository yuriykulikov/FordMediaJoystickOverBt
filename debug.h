/*
 * debug.h
 *
 *  Created on: 30.06.2013
 *      Author: Yuriy
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include "usart_driver_RTOS.h"
USART_buffer_struct_t FTDI_USART;
void logs(const char *string);
void logi(int16_t Int,int16_t radix);

#endif /* DEBUG_H_ */
