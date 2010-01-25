#ifndef _TCA6416_KEYS_H
#define _TCA6416_KEYS_H

#include <linux/gpio_keys.h>
#include <linux/types.h>

struct tca6416_keys_platform_data {
	struct gpio_keys_button * buttons;
	int nbuttons;
	unsigned int rep:1;	/* enable input subsystem auto repeat */
	uint16_t pinmask;
	uint16_t invert;
	int irqnum;
	int irq_is_gpio;
	int use_polling;	/* use polling if Interrupt is not connected*/
};

#endif
