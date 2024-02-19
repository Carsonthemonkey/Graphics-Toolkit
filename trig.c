#include <math.h>
#include "trig.h"

double to_radians(double degrees){
    return degrees * (M_PI / 180);
}

double to_degrees(double radians){
    return radians * (180 / M_PI);
}
