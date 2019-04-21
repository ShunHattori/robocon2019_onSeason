/*#include "QEI.h"

Serial pc(USBTX, USBRX);
Timer QEITimer;
//Use X4 encoding.
//QEI wheel(p29, p30, NC, 624, QEI::X4_ENCODING);
//Use X2 encoding by default.
QEI encoder_XAxis_1(PB_5, PC_7, NC, 48, &QEITimer, QEI::X4_ENCODING);
QEI SUBencoder(PC_6, PB_15, NC, 48, &QEITimer);
QEI encoder_YAxis_1(PF_13, PE_9, NC, 48, &QEITimer);
int main()
{

    while (1)
    {
        wait(0.1);
        pc.printf("Pulses is: %i\n\r", encoder_XAxis_1.getPulses());
    }
}*/