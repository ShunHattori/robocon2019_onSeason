#include "mbed.h"
#include "encoder.h"

InterruptIn button(PG_0);

Serial LCD(PD_5, PD_6);
Serial PC(USBTX, USBRX);

Ticker led1t;
Ticker led2t;
Ticker led3t;
Ticker LCDUpdate;

DigitalOut led1(PB_0);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(PD_1);
AnalogIn pot(PF_7);
PwmOut pwmPin(PF_9);

uint64_t whileCount = 0;
unsigned char data[4] = {0xFE, 0x51, 0xFE, 0x46};

double delay = 0.5; // 500 ms

void flip1()
{
    led1 = !led1;
}
void flip2()
{
    led2 = !led2;
}
void flip3()
{
    led3 = !led3;
}
void pressed()
{
    led4 = 1;
}

void released()
{
    led4 = 0;
}

void LCDupdate()
{
    LCD.putc(data[0]);
    LCD.putc(data[1]);
    LCD.putc(data[2]);
    LCD.putc(data[3]);
    //LCD.printf("current:%.3f%%", pot.read());
    pwmPin.write(pot.read());
}

int main()
{
    // Assign functions to button
    button.rise(&pressed);
    button.fall(&released);

    led1t.attach(&flip1, 0.015);
    wait(0.01);
    led2t.attach(&flip2, 0.015);
    wait(0.01);
    led3t.attach(&flip3, 0.015);
    wait(0.01);
    PC.baud(115200);
    LCD.baud(9600);
    LCD.putc(data[0]);
    LCD.putc(data[1]);
    LCD.putc(data[2]);
    LCD.putc(data[3]);
    LCD.printf("Hello from STM32F767");
    pwmPin.period_us(10);
    pwmPin.write(0.50f);

    LCDUpdate.attach(&LCDupdate, 1);
    while (1)
    {
    }
}
