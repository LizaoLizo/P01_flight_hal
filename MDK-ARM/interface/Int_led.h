#ifndef __INT_LED__
#define __INT_LED__

#include <main.h>

typedef struct 
{
    GPIO_TypeDef * port;
    uint16_t pin;
    /* data */
}LED_Struct;

void Int_led_turn_on(LED_Struct * led);

void Int_led_turn_off(LED_Struct * led);

void Int_led_toggle(LED_Struct * led);




#endif 