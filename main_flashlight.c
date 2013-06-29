#include "avr_compiler.h"
#include "main.h"
#include "Handler.h"
#include "Leds.h"
#include "Buttons.h"

Message msgArray[QUEUE_MAX_LEN];
LedGroup leds;
Led red;
Led green;
Led blue;
Led ledArray[3];
Button_struct_t plusButton;
Button_struct_t minusButton;
uint8_t brightness = 0;
Handler handler;

typedef enum {
	CHECK_BUTTONS = 1,
	SET_OFF = 3,
	HEARTBEAT = 4,
	HEARTBEAT_OFF = 5,
	SETUP = 6,
} Message_code_t;

ISR(TIM0_COMPA_vect) {
	tick++;
}

void initLeds() {
	Led_init(&red, &PORTB, &DDRB, (1 << PB0), 1);
	Led_init(&green, &PORTB, &DDRB, (1 << PB1), 1);
	Led_init(&blue, &PORTB, &DDRB, (1 << PB2), 1);
	LedGroup_init(&leds, ledArray);
	LedGroup_add(&leds, &red);
	LedGroup_add(&leds, &green);
	LedGroup_add(&leds, &blue);
	LedGroup_set(&leds, NONE);
}

void Brightness_set(uint8_t mode) {
	uint8_t pwm = 0;
	switch (mode) {
	case 0:
		pwm = 0;
		PORTA &= ~(1 << PA1);			//!SHDN enable the device
		TCCR1A &= ~(1 << WGM10) | (1 << WGM11);			//Stop PWM to save power
		break;
	case 1:
		pwm = 10;
		break;
	case 2:
		pwm = 15;
		break;
	case 3:
		pwm = 31;
		break;
	case 4:
		pwm = 63;
		break;
	case 5:
		pwm = 127;
		break;
	case 6:
		pwm = 255;
		break;
	default:
		pwm = 0;
		break;
	}
	if (pwm != 0) {
		OCR1B = pwm;			//Set mode
		TCCR1A |= (1 << WGM10) | (1 << WGM11);			//Start PWM Timer
		PORTA |= (1 << PA1);			//!SHDN enable the device and mosfetS
	}
}

void onMinusClick() {
	LedGroup_set(&leds, GREEN);
	Message *message = Handler_obtain(&handler, SET_OFF);
	Handler_sendMessageDelayed(&handler, message, 198);
	if ((brightness > 1) && (brightness <= 6)) {
		brightness--;
		Brightness_set(brightness);
	}
}

void onPlusClick() {
	LedGroup_set(&leds, GREEN);
	Message *message = Handler_obtain(&handler, SET_OFF);
	Handler_sendMessageDelayed(&handler, message, 198);
	if ((brightness > 0) && (brightness < 6)) {
		brightness++;
		Brightness_set(brightness);
	}
}

void onLongClick() {
	LedGroup_set(&leds, RED);
	Message *message = Handler_obtain(&handler, SET_OFF);
	Handler_sendMessageDelayed(&handler, message, 198);
	if (brightness == 0) {
		brightness = 3;
	} else {
		brightness = 0;
	}
	Brightness_set(brightness);
}

inline void configureTickTimer(void) {

	TCCR0A &= 0x00;			//stop counter, clear all control bits
	TCCR0B &= 0x00;			//stop counter, clear all control bits
	TCNT0 = 0x0000;			//clear counter
	TCCR0B |= (1 << WGM02);			//CTC mode clear on compare
	TIMSK0 |= (1 << OCIE0A);			//enable compa_vect
#if (F_CPU/TICK_FREQUENCY>0xFFFF)
#error "hey thats too slow!"
#endif
	OCR0A = (F_CPU / TICK_FREQUENCY);
	TCCR0B |= TIMER1_PREDIVISOR_1;
	tick = 0;
}

inline void initPorts() {
	DDRA |= (1 << PA1);			//!SHDN
	DDRA |= (1 << PA7);			//ctrl output
	PORTA |= (1 << PA7);			//ctrl always eneable

	DIDR0 &= 0x01;			//voltage adc pin

	//start PWM
	TCCR1A |= (1 << COM1B1);//OC0B non-inverting mode OCR0B OCR0B=0xFF goes for continious high, OCR0B=0x00 = OFF
	TCCR1B |= (1 << CS10);			//no prescaler for high frequency
	DDRA |= (1 << PA5);			//PWM
	Brightness_set(0);
	brightness = 0;
}

void handleMessage(Message msg, void *context, Handler *handler) {
	switch (msg.what) {
	case CHECK_BUTTONS:
		Button_checkButton(&plusButton);
		Button_checkButton(&minusButton);
		Message *buttonMessage = Handler_obtain(handler, CHECK_BUTTONS);
		Handler_sendMessageDelayed(handler, buttonMessage, BUTTON_CHECK_PERIOD);
		break;
	case SET_OFF:
		LedGroup_set(&leds, NONE);
		break;
	case HEARTBEAT:
		LedGroup_set(&leds, BLUE);
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

int main(void) {
	initPorts();
	configureTickTimer();
	initLeds();

	Button_init(&plusButton, &PINA, &PORTA, (1 << PA2), onPlusClick, onLongClick);
	Button_init(&minusButton, &PINA, &PORTA, (1 << PA4), onMinusClick, onLongClick);

	MsgQueue queue;
	MsgQueue_init(&queue, msgArray, 10);
	Handler_init(&handler, &queue, handleMessage, 0);

	Handler_sendEmptyMessage(&handler, CHECK_BUTTONS);
	Handler_sendEmptyMessage(&handler, SETUP);
	//Handler_sendEmptyMessage(&handler, HEARTBEAT);

	/* Enable interrupts */sei();
	/* Enter the superloop which manages tasks */
	while (1) {
		MsgQueue_processNextMessage(&queue);
	}
	return 0;	//Should never get here
}

/* Define tasks here. */
