#include <mbed.h>

static const uint8_t led_mask[] = {
0,0b00000001,0b00000010,0b00000100,0b00000010,
0b00000001,0,0b00000111,0b00000101,0b00000010};
BusOut leds(LED1,LED2,LED3);

int main()
{
    int k = sizeof(led_mask), n = 0;
    while(1)
    {
        leds = led_mask[n%k];
        n++;
        ThisThread::sleep_for(1000);
    }
}