#include "avr_compiler.h"
#include "Leds.h"
#include <avr/sleep.h>
#include "FreeRTOS.h"
#include "clksys_driver.h"
#include "pmic_driver.h"
#include "usart_driver_RTOS.h"
#include "Handler.h"
#include "Buttons.h"
#include "port_driver.h"

Message msgArray[QUEUE_MAX_LEN];
LedGroup leds;
Led red;
Led green;
Led blue;
Led ledArray[3];
Button_struct_t plusButton;
Handler handler;
MsgQueue queue;
USART_buffer_struct_t FTDI_USART;

typedef enum s{
    RED = 0x01,
    GREEN = 0x02,
    BLUE= 0x04,

    ORANGE = 0x03,
    PINK = 0x05,
    SKY = 0x06,

    WHITE = 0x07,
    NONE = 0x00,
} Color_enum;

typedef enum {
    CHECK_BUTTONS = 1,
    SET_OFF = 3,
    HEARTBEAT = 4,
    HEARTBEAT_OFF = 5,
    SETUP = 6,
} Message_code_t;

/* BADISR_vect is called when interrupt has occurred, but there is no ISR handler for it defined */
ISR (BADISR_vect){
	//stop execution and report error
	//while(true) LED_set(ORANGE);
}

void initLeds() {
    PORT_ConfigurePins(&PORTF, PIN0_bm, 0, 0, PORT_OPC_WIREDANDPULL_gc, PORT_ISC_BOTHEDGES_gc );
    PORT_ConfigurePins(&PORTF, PIN1_bm, 0, 0, PORT_OPC_WIREDANDPULL_gc, PORT_ISC_BOTHEDGES_gc );
    PORT_ConfigurePins(&PORTF, PIN6_bm, 0, 0, PORT_OPC_WIREDANDPULL_gc, PORT_ISC_BOTHEDGES_gc );
    PORT_SetPinAsOutput(&PORTF, PIN0_bp);
    PORT_SetPinAsOutput(&PORTF, PIN1_bp);
    PORT_SetPinAsOutput(&PORTF, PIN6_bp);
    //PORTF.OUT = 0;

    //while(1);

    Led_init(&red, &PORTF.OUT, PIN0_bm, 1);
    Led_init(&blue, &PORTF.OUT, PIN1_bm, 1);
    Led_init(&green, &PORTF.OUT, PIN6_bm, 1);

    LedGroup_init(&leds, ledArray);
    LedGroup_add(&leds, &red);
    LedGroup_add(&leds, &green);
    LedGroup_add(&leds, &blue);
    LedGroup_set(&leds, NONE);
}

void onPlusClick() {
    LedGroup_set(&leds, WHITE);
    Message *message = Handler_obtain(&handler, SET_OFF);
    Handler_sendMessageDelayed(&handler, message, 198);
//TODO
}

void onLongClick() {
    LedGroup_set(&leds, WHITE);
    Message *message = Handler_obtain(&handler, SET_OFF);
    Handler_sendMessageDelayed(&handler, message, 198);
    //TODO
    USART_Buffer_PutString(&FTDI_USART, "onLongClick\n",DONT_BLOCK);
}

void handleMessage(Message msg, void *context, Handler *handler) {
    switch (msg.what) {
    case CHECK_BUTTONS:
        Button_checkButton(&plusButton);
//TODO
        Message *buttonMessage = Handler_obtain(handler, CHECK_BUTTONS);
        Handler_sendMessageDelayed(handler, buttonMessage, CHECK_BUTTON_PERIOD);
        break;
    case SET_OFF:
        LedGroup_set(&leds, NONE);
        break;
    case HEARTBEAT:
        LedGroup_set(&leds, WHITE);
        Message *message = Handler_obtain(handler, HEARTBEAT_OFF);
        Handler_sendMessageDelayed(handler, message, 1000);
        break;
    case HEARTBEAT_OFF:
        LedGroup_set(&leds, NONE);
        Message *messageHBOff = Handler_obtain(handler, HEARTBEAT);
        Handler_sendMessageDelayed(handler, messageHBOff, 1000);
        break;
    }
}

void doSchedulerTask(){
    while (1) {
        MsgQueue_processNextMessage(&queue);
    }
}

int main( void )
{
	/*  Enable internal 32 MHz ring oscillator and wait until it's
	 *  stable. Set the 32 MHz ring oscillator as the main clock source.
	 */
	CLKSYS_Enable( OSC_RC32MEN_bm );
	CLKSYS_Prescalers_Config( CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc );
	do {} while ( CLKSYS_IsReady( OSC_RC32MRDY_bm ) == 0 );
	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC32M_gc );
	/* Do all configuration and create all tasks and queues before scheduler is started.
	 * It is possible to put initialization of peripherals like displays into task functions
	 * (which will be executed after scheduler has started) if fast startup is needed.
	 * Interrupts are not enabled until the call of vTaskStartScheduler();
	 */
    MsgQueue_init(&queue, msgArray, 20);
    Handler_init(&handler, &queue, handleMessage, 0);

    initLeds();

    PORT_ConfigurePins(&PORTE, PIN5_bm, 0, 0 ,PORT_OPC_PULLUP_gc, PORT_ISC_FALLING_gc );


    Button_init(&plusButton, &PORTE.IN, PIN5_bm, onPlusClick, onLongClick);
    LedGroup_set(&leds, WHITE);
    Message *message = Handler_obtain(&handler, SET_OFF);
    Handler_sendMessageDelayed(&handler, message, 1905);


    Handler_sendEmptyMessage(&handler, CHECK_BUTTONS);
    Handler_sendEmptyMessage(&handler, SETUP);
  //  Handler_sendEmptyMessage(&handler, HEARTBEAT);

	FTDI_USART = USART_InterruptDriver_Initialize(&USARTD0, BAUD9600, 64);
	/* Report itself. */
//
	/* Start USART task */
//	xTaskCreate(vUSARTTask, ( signed char * ) "USARTTSK", 1000,&FTDI_USART, configNORMAL_PRIORITY, NULL );
	/* Start LED task for testing purposes */
	xTaskCreate(doSchedulerTask, ( signed char * ) "SDLR", configMINIMAL_STACK_SIZE+500, NULL, configNORMAL_PRIORITY, NULL );
	/* Start SPISPY task */
	//vStartSPISPYTask(configNORMAL_PRIORITY);

//	LED_queue_put(debugLed,BLUE,700);
//	LED_queue_put(debugLed,SKY,700);
//	LED_queue_put(debugLed,WHITE,700);
	/* Enable PMIC interrupt level low. */


	PMIC_EnableLowLevel();
	/* Start scheduler. Creates idle task and returns if failed to create it.
	 * vTaskStartScheduler never returns during normal operation. If it has returned, probably there is
	 * not enough space in heap to allocate memory for the idle task, which means that all heap space is
	 * occupied by previous tasks and queues. Try to increase heap size configTOTAL_HEAP_SIZE in FreeRTOSConfig.h
	 * XMEGA port uses heap_1.c which doesn't support memory free. It is unlikely that XMEGA port will need to
	 * dynamically create tasks or queues. To ensure stable work, create ALL tasks and ALL queues before
	 * vTaskStartScheduler call. In this case we can be sure that heap size is enough.
	 * Interrupts would be enabled by calling sei();*/
	vTaskStartScheduler();

	/* stop execution and report error */
//	LED_set(PINK);
	//while(true) LED_set(PINK);
	return 0;
}
void vApplicationIdleHook( void )
{
   /* Go to sleep mode if there are no active tasks	*/
//	sleep_mode();
}
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName )
{
	/* stop execution and report error */
	//while(true) LED_set(RED);
}

void vApplicationTickHook( void )
{
    tick++;
}
