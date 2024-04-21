#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <stdbool.h>
#include "random.h"
#include "vector.h"

void rand_init() {
    srand(time(NULL)); 
}

double rand_double() {
    return (double)rand() / (double)RAND_MAX;
}

Vector3 random_point_in_sphere(double sphere_radius){
    double theta = 2 * M_PI * rand_double();
    double phi = acos(2 * rand_double() - 1);
    double x = sphere_radius * sin(phi) * cos(theta);
    double y = sphere_radius * sin(phi) * sin(theta);
    double z = sphere_radius * cos(phi);
    return (Vector3){x, y, z};
}

Vector2 random_point_in_circle(double circle_radius){
    double r = circle_radius * sqrt(rand_double());
    double theta = 2 * M_PI * rand_double();
    double x = r * cos(theta);
    double y = r * sin(theta);
    return (Vector2){x, y};
}
