#include "debug.h"

void logs(const char *string){
    USART_Buffer_PutString(&FTDI_USART, string, DONT_BLOCK);
}
void logi(int16_t Int,int16_t radix){
    USART_Buffer_PutInt(&FTDI_USART, Int, radix, DONT_BLOCK);
}
