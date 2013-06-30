#ifndef ADC_H_
#define ADC_H_

#include "avr_compiler.h"
#include "adc_driver.h"

#define SEEK_UP_bm 0x01
#define SEEK_DOWN_bm 0x02
#define VOLUME_UP_bm 0x04
#define VOLUME_DOWN_bm 0x08
#define MODE_bm 0x10
#define BUTTONS_gm 0x1F

void doADCTask(uint8_t *button);

#endif /* ADC_H_ */
