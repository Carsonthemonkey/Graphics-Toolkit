#ifndef MATRIX_H
#define MATRIX_H
#include "vector.h"


#define IDENTITY_INITIALIZER {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}

/**
 * @brief Sets the given 4x4 matrix to the identity matrix. This function modifies the input matrix by setting all elements to 0, except for the diagonal elements which are set to 1.
 * 
 * @param matrix The matrix to be made into an identity. The contents of the matrix are overwritten.
 */
void mat4_make_identity(double matrix[4][4]);

// /**
//  * @brief Translates the provided matrix by the x, y, and z values.
//  * 
//  * @param matrix The matrix to be translated. It is modified in place
//  * @param translation A Vectr representing how the matrix should be translated
//  */
// void mat4_translate_matrix(double matrix[4][4], Vector3 translation);

// /**
//  * @brief Scales the provided matrix by the x, y, and z values.
//  * 
//  * @param matrix The matrix to be scaled. It is modified in place
//  * @param x The x-axis scale
//  * @param y The y-axis scale
//  * @param z The z-axis scale
//  */
// void mat4_scale_matrix(double matrix[4][4], double x, double y, double z);

// /**
//  * @brief Rotates a matrix on the x-axis
//  * 
//  * @param matrix The matrix to be rotated. It is modified in place
//  * @param x_degrees The degrees on the x axis to be rotated
//  */
// void mat4_rotate_matrix_x_degrees(double matrix[4][4], double x_degrees);

// /**
//  * @brief Rotates a matrix on the y-axis
//  * 
//  * @param matrix The matrix to be rotated. It is modified in place
//  * @param x_degrees The degrees on the y axis to be rotated
//  */
// void mat4_rotate_matrix_y_degrees(double matrix[4][4], double y_degrees);

// /**
//  * @brief Rotates a matrix on the z-axis
//  * 
//  * @param matrix The matrix to be rotated. It is modified in place
//  * @param x_degrees The degrees on the z axis to be rotated
//  */
// void mat4_rotate_matrix_z_degrees(double matrix[4][4], double z_degrees);

/* Vector Transformations */

/**
 * @brief Transforms a 3D point according to a transformation matrix
 * 
 * @param point The point to be transformed
 * @param transform A 4x4 transform matrix to be applied to the point
 * @return Vector3 The resulting transformed point
 */
Vector3 mat4_mult_point(Vector3 point, double transform[4][4]);

/**
 * @brief Transforms an array of points according to a transformation matrix
 * 
 * @param points An array of points to be transformed. This array will be modified in place.
 * @param transform A 4x4 transform matrix to be applied to the points
 * @param num_points The number of points to be transformed
 */
void mat4_mat_mult_points(Vector3* points, double transform[4][4], int num_points);

#endif