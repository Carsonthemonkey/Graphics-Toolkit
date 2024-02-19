#include "matrix.h"


Vector3 mat4_mult_point(Vector3 point, double transform[4][4]){
    Vector3 result;
    result.x = transform[0][0] * point.x + transform[0][1] * point.y + transform[0][2] * point.z + transform[0][3];
    result.y = transform[1][0] * point.x + transform[1][1] * point.y + transform[1][2] * point.z + transform[1][3];
    result.z = transform[2][0] * point.x + transform[2][1] * point.y + transform[2][2] * point.z + transform[2][3];
    return result;
}