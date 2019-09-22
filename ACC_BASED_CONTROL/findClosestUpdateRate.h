#ifndef FIND_CLOSEST_UPDATE_RATE_H
#define FIND_CLOSEST_UPDATE_RATE_H

#include <xsens/xsintarray.h>
#include <sstream>

int findClosestUpdateRate(const XsIntArray& supportedUpdateRates, const int desiredUpdateRate);

#endif /* FIND_CLOSEST_UPDATE_RATE_H */