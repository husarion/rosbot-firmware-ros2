#include <mbed.h>

Serial serial(SENS3_PIN3_UART_TX, SENS3_PIN4_UART_RX, 230400);

DigitalOut led(LED1);

int test()
{
    while(1)
    {
        led = !led;
        serial.printf("Hello World!\r\n");
        ThisThread::sleep_for(1000);
    }
}