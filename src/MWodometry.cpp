#include "MWodometry.h"

MWodometry::~MWodometry(void)
{
    printf("Instance removed.");
}

long MWodometry::getDistance(void) const
{
    return ((2 * M_PI * wheelRadius) * encObject->getPulse() / encoderResolution);
}

bool MWodometry::setDistance(long overloadDistance)
{
    encObject->setPulse(long((overloadDistance * encoderResolution) / (2 * M_PI * wheelRadius)));
    return 1;
}