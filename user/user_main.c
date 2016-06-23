/*
 *  Example testing button and LED
 *  
 *  Button 1 is connected to GPIO0 (Pin 3) ESP-01
 *  LED is connected to GPIO2 (Pin 4) ESP-01
 *
 *  Pin number:
 *  -----------
 *  Pin 0 = GPIO16
 *  Pin 1 = GPIO5
 *  Pin 2 = GPIO4
 *  Pin 3 = GPIO0
 *  Pin 4 = GPIO2
 *  Pin 5 = GPIO14
 *  Pin 6 = GPIO12
 *  Pin 7 = GPIO13
 *  Pin 8 = GPIO15
 *  Pin 9 = GPIO3
 *  Pin 10 = GPIO1
 *  Pin 11 = GPIO9
 *  Pin 12 = GPIO10
 *
 */

#include "ets_sys.h"
#include "osapi.h"
#include "os_type.h"
#include "user_interface.h"
#include "driver/uart.h"
#include "driver/gpio16.h"

extern int ets_uart_printf(const char *fmt, ...);
int (*console_printf)(const char *fmt, ...) = ets_uart_printf;

extern uint8_t pin_num[GPIO_PIN_NUM];

#define GPIO_ZERO_PIN 1 // GPIO5
#define GPIO_LAMP_PIN 2 // GPIO4
#define USE_US_TIMER

// GPIO_PIN_INTR_NEGEDGE - down
// GPIO_PIN_INTR_POSEDGE - up
// GPIO_PIN_INTR_ANYEDGE - both
// GPIO_PIN_INTR_LOLEVEL - low level
// GPIO_PIN_INTR_HILEVEL - high level
// GPIO_PIN_INTR_DISABLE - disable interrupt
const char *gpio_type_desc[] =
{
	    "GPIO_PIN_INTR_DISABLE (DISABLE INTERRUPT)",
	    "GPIO_PIN_INTR_POSEDGE (UP)",
	    "GPIO_PIN_INTR_NEGEDGE (DOWN)",
	    "GPIO_PIN_INTR_ANYEDGE (BOTH)",
	    "GPIO_PIN_INTR_LOLEVEL (LOW LEVEL)",
	    "GPIO_PIN_INTR_HILEVEL (HIGH LEVEL)"
};

#define DELAY 3000 /* microseconds */
#define DELAYOFF 3150 /* microseconds */

LOCAL os_timer_t key_timer;
LOCAL os_timer_t key_timer_off;

LOCAL void ICACHE_FLASH_ATTR key_cb(void *arg) {
	gpio_write(GPIO_LAMP_PIN, 1);
}

LOCAL void ICACHE_FLASH_ATTR key_cb_off(void *arg) {
	gpio_write(GPIO_LAMP_PIN, 0);
}

void ICACHE_FLASH_ATTR intr_callback(unsigned pin, unsigned level) {
	gpio_write(GPIO_LAMP_PIN, 0);
	os_timer_disarm(&key_timer);
	os_timer_disarm(&key_timer_off);
	os_timer_setfn(&key_timer, (os_timer_func_t *)key_cb, (void *)0); // set function to a timer
	os_timer_setfn(&key_timer_off, (os_timer_func_t *)key_cb_off, (void *)0); // set function to a timer
	ets_timer_arm_new(&key_timer, DELAY, 0, 0);
	ets_timer_arm_new(&key_timer_off, DELAYOFF, 0, 0);
}

void user_rf_pre_init(void){}

void user_init(void) {
	system_timer_reinit();
	GPIO_INT_TYPE gpio_type;

	UARTInit(BIT_RATE_230400);
	console_printf("\r\n");

	// Set Wifi softap mode
	wifi_station_disconnect();
	wifi_station_set_auto_connect(0);
	wifi_set_opmode(SOFTAP_MODE);

	if (set_gpio_mode(GPIO_LAMP_PIN, GPIO_PULLDOWN, GPIO_OUTPUT)) {
			console_printf("GPIO%d set GPIO_OUTPUT mode\r\n", pin_num[GPIO_LAMP_PIN]);
		} else {
			console_printf("Error: GPIO%d not set GPIO_OUTPUT mode\r\n", pin_num[GPIO_LAMP_PIN]);
	}

	gpio_write(GPIO_LAMP_PIN, 1);

	gpio_type = GPIO_PIN_INTR_POSEDGE;

	if (set_gpio_mode(GPIO_ZERO_PIN, GPIO_PULLUP, GPIO_INT)) {
		console_printf("GPIO%d set interrupt mode\r\n", pin_num[GPIO_ZERO_PIN]);
		if (gpio_intr_init(GPIO_ZERO_PIN, gpio_type)) {
			console_printf("GPIO%d enable %s mode\r\n", pin_num[GPIO_ZERO_PIN], gpio_type_desc[gpio_type]);
			gpio_intr_attach(intr_callback);
		} else {
			console_printf("Error: GPIO%d not enable %s mode\r\n", pin_num[GPIO_ZERO_PIN], gpio_type_desc[gpio_type]);
		}
	} else {
		console_printf("Error: GPIO%d not set interrupt mode\r\n", pin_num[GPIO_ZERO_PIN]);
	}
}
