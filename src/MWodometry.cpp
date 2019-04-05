#include "MWodometry.h"

MWodometry::~MWodometry(void)
{
    printf("Instance removed.");
}

double MWodometry::getDistance(void) const
{
    return ((2 * M_PI * wheelRadius) * encObject->getPulse() / encoderResolution);
}

void MWodometry::setDistance(double overloadDistance)
{
    encObject->setPulse(double((overloadDistance * encoderResolution) / (2 * M_PI * wheelRadius)));
}