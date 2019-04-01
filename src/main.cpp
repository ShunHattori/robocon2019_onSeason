#include "mbed.h"
#include "Encoder.h"

Encoder encoder_XAxis_1(PG_4, PG_7);
Serial PC(USBTX, USBRX);
DigitalOut LED(LED1);
InterruptIn button(USER_BUTTON);

void button_pressed(){
    PC.printf("Pulse:%ld\r\n",encoder_XAxis_1.getPulse());
}

int main()
{
    PC.baud(9600);
    button.mode(PullDown);
    button.rise(&button_pressed);
    encoder_XAxis_1.setPulse(0);
    PC.printf("got encoder pulse:%ld\r\n", encoder_XAxis_1.getPulse());
    LED = 1;

    while (1)
    {
        static long dammy_pulse_temp = 0;
        encoder_XAxis_1.setPulse(dammy_pulse_temp++);
        wait(0.02);
    }
}
