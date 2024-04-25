/**
 * @file vector.h
 * @author Carson Reader
 * @brief A library of functions for working with 2D and 3D vectors.
 * @version 0.1
 * @date 2024-01-23
 */

#ifndef VECTOR_H
#define VECTOR_H
#include <stdbool.h>

//These allows vectors to be used for FPToolkit args more easily
#define SPREAD_VEC3(v) (v).x, (v).y, (v).z
#define SPREAD_VEC2(v) (v).x, (v).y

/**
 * @struct Vector3
 * @brief Represents a 3D vector with x, y, and z components.
 */
typedef union Vector3 {
    struct{
        double x;
        double y;
        double z;
    };
    struct {
        double r;
        double g;
        double b;
    };
} Vector3;

extern const Vector3 VEC3_ZERO, VEC3_FORWARD, VEC3_BACK, VEC3_UP, VEC3_DOWN, VEC3_LEFT, VEC3_RIGHT;
/**
 * Prints the components of a Vector3.
 *
 * @param v The Vector3 to be printed.
 */
void vec3_print(Vector3 v);

/**
 * @brief Checks two vectors for equality
 * 
 * @param a The first vector
 * @param b The second vector
 * @return true If the vectors are equal
 * @return false If the vevtors are inequal
 */
bool vec3_is_equal(Vector3 a, Vector3 b);

/**
 * Calculates the magnitude of a 3D vector.
 *
 * @param v The vector for which to calculate the magnitude.
 * @return The magnitude of the vector.
 */
double vec3_magnitude(Vector3 v);

/**
 * @brief Normalize a 3D vector.
 *
 * This function normalizes a given 3D vector, making its length equal to 1.
 *
 * @param v Pointer to the Vector3 structure representing the vector to be normalized.
 */
void vec3_normalize(Vector3 *v);

/**
 * Normalizes a Vector3.
 *
 * This function takes a Vector3 as input and returns a normalized version of it.
 * The normalized vector has the same direction as the original vector, but its length is 1.
 *
 * @param v The Vector3 to be normalized.
 * @return The normalized Vector3.
 */
Vector3 vec3_normalized(Vector3 v);

/**
 * Adds two 3D vectors.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The sum of the two vectors.
 */
Vector3 vec3_add(Vector3 a, Vector3 b);

/**
 * Subtracts two vectors.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The resulting vector after subtracting b from a.
 */
Vector3 vec3_sub(Vector3 a, Vector3 b);

/**
 * Negates a Vector3 by changing the sign of each component.
 *
 * @param v The Vector3 to be negated.
 * @return The negated Vector3.
 */
Vector3 vec3_negated(Vector3 v);

/**
 * Multiplies two Vector3 objects element-wise.
 *
 * @param a The first Vector3 object.
 * @param b The second Vector3 object.
 * @return The resulting Vector3 object after element-wise multiplication.
 */
Vector3 vec3_mult(Vector3 a, Vector3 b);

/**
 * Divides two vectors element-wise.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The resulting vector after element-wise division.
 */
Vector3 vec3_div(Vector3 a, Vector3 b);

/**
 * Scales a Vector3 by a given scale factor.
 *
 * @param v The Vector3 to be scaled.
 * @param scale The scale factor.
 * @return The scaled Vector3.
 */
Vector3 vec3_scale(Vector3 v, double scale);

/**
 * Calculates the dot product of a 3D vector.
 *
 * @param v The vector for which to calculate the dot product.
 * @return The dot product of the vector.
 */
double vec3_dot_prod(Vector3 a, Vector3 b);

/**
 * Calculates the cross product of two 3D vectors.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The cross product of the two vectors.
 */
Vector3 vec3_cross_prod(Vector3 a, Vector3 b);

/**
 * @brief Calculates the distance between two 3D vectors
 * 
 * @param a The first vector to calculat the distance
 * @param b The second vector to calculate the distance
 * @return double the distance between the vectors
 */
double vec3_distance(Vector3 a, Vector3 b);

/**
 * @brief Linearly interpolates between two 3D vectors
 * 
 * @param a The first vector
 * @param b The second vector
 * @param t The interpolation factor
 * @return Vector3 The interpolated vector
 */
Vector3 vec3_lerp(Vector3 a, Vector3 b, double t);

/**
 * @brief Computes the reflection vector given the incoming and normal vector
 * 
 * @param incoming The incoming vector
 * @param normal The normal vector of the reflection plane
 * @return Vector3 The reflection vector
 */
Vector3 vec3_reflection(Vector3 incoming, Vector3 normal);

typedef union {
    struct {
        float x;
        float y;
        float z;
    };
    struct {
        float r;
        float g;
        float b;
    };
} Vector3f;

/**
 * @brief Construct a new floating point vector from a double precision vector
 * 
 * @param v The double precision vector to convert to a floating point vector
 */
Vector3f vec3_to_vec3f(Vector3 v);

/**
 * @brief Construct a new double precision vector from a floating point vector
 * 
 * @param v The floating point vector to convert to a double precision vector
 */
Vector3 vec3f_to_vec3(Vector3f v);

/**
 * Scales a Vector3f by a given scale factor.
 *
 * @param v The Vector3 to be scaled.
 * @param scale The scale factor.
 * @return The scaled Vector3.
 */
Vector3f vec3f_scale(Vector3f v, double scale);

/**
 * Adds two 3D vectors.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The sum of the two vectors.
 */
Vector3f vec3f_add(Vector3f a, Vector3f b);


double* vec3_to_array(Vector3* vec);

/**
 * @struct Vector2
 * @brief Represents a 2D vector with x and y components.
 */
typedef struct Vector2 {
    double x; 
    double y; 
} Vector2;

extern const Vector2 VEC2_ZERO, VEC2_UP, VEC2_DOWN, VEC2_LEFT, VEC2_RIGHT;

/**
 * Prints the components of a 2D vector.
 *
 * @param v The vector to be printed.
 */
void vec2_print(Vector2 v);

/**
 * Calculates the magnitude of a 2D vector.
 *
 * @param v The 2D vector.
 * @return The magnitude of the vector.
 */
double vec2_magnitude(Vector2 v);

/**
 * @brief Normalize a 2D vector.
 *
 * This function normalizes a 2D vector, making its length equal to 1.
 *
 * @param v The vector to be normalized.
 */
void vec2_normalize(Vector2 *v);

/**
 * Calculates the normalized vector of a given Vector2.
 *
 * @param v The Vector2 to be normalized.
 * @return The normalized Vector2.
 */
Vector2 vec2_normalized(Vector2 v);

/**
 * Adds two 2D vectors.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The sum of the two vectors.
 */
Vector2 vec2_add(Vector2 a, Vector2 b);

/**
 * Subtracts two 2D vectors.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The result of subtracting vector b from vector a.
 */
Vector2 vec2_sub(Vector2 a, Vector2 b);

/**
 * Multiplies two 2D vectors element-wise.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The resulting vector after element-wise multiplication.
 */
Vector2 vec2_mult(Vector2 a, Vector2 b);
/**
 * Divides two 2D vectors.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The result of dividing vector a by vector b.
 */
Vector2 vec2_div(Vector2 a, Vector2 b);

/**
 * @brief Scales a 2D vector by a given scale factor.
 *
 * This function multiplies both the x and y components of the vector by the given scale factor.
 *
 * @param scale The scale factor to multiply the vector by.
 * @return The scaled vector.
 */
Vector2 vec2_scale(Vector2 v, double scale);

/**
 * Calculates the dot product of a 2D vector.
 *
 * @param v The 2D vector.
 * @return The dot product of the vector.
 */
double vec2_dot_prod(Vector2 a, Vector2 b);


#endif