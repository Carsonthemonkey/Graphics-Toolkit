#ifndef RANDOM_H
#define RANDOM_H
#include "vector.h"
/**
 * @brief Initializes random number generation
 * 
 */
void rand_init();

/**
 * @brief Returns a random double between 0 and 1
 * 
 * @return double A random double between 0 and 1
 */
double rand_double();

/**
 * @brief Returns a random point in a sphere
 * 
 * @param sphere_radius The radius of the sphere to find a point inside of
 * @return Vector3 The random point in the sphere
 */
Vector3 random_point_in_sphere(double sphere_radius);

/**
 * @brief Returns a random point ina sphere
 * 
 * @param circle_radius The radius of the circle to find a point inside of
 * @return Vector2 The random point in the circle
 */
Vector2 random_point_in_circle(double circle_radius);
#endif