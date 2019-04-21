#include "MWodometry.h"

MWodometry::~MWodometry(void)
{
    printf("Instance removed.");
}

double MWodometry::getDistance(void)
{
    return ((2 * M_PI * wheelRadius) * QEIobj->getPulses() / encoderResolution);
}

void MWodometry::setDistance(double overloadDistance)
{
    QEIobj->qei_reset();
    //QEIobj->setPulse(double((overloadDistance * encoderResolution) / (2 * M_PI * wheelRadius)));
}