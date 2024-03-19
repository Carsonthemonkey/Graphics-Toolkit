#ifndef M3D_MATRIX_TOOLS_H
#define M3D_MATRIX_TOOLS_H

#define SX 0
#define SY 1
#define SZ 2

#define RX 3
#define RY 4
#define RZ 5

#define TX 6
#define TY 7
#define TZ 8

#define NX 9
#define NY 10
#define NZ 11

/**
 * Prints the elements of a 4x4 matrix.
 *
 * @param a The matrix to be printed.
 */
int M3d_print_mat(double a[4][4]);

/**
 * Copies the contents of a 4x4 matrix into another 4x4 matrix.
 *
 * @param out The destination matrix where the contents will be copied to.
 * @param matrix The source matrix from which the contents will be copied.
 */
int M3d_copy_mat(double out[4][4], double matrix[4][4]);

/**
 * @brief Sets the given 4x4 matrix to the identity matrix.
 *
 * This function modifies the input matrix by setting all elements to 0, except for the diagonal elements which are set to 1.
 *
 * @param a The 4x4 matrix to be set to the identity matrix.
 */
int M3d_make_identity(double a[4][4]);

/**
 * Makes a translation matrix.
 *
 * This function creates a translation matrix that translates points in 3D space
 * by the specified amounts along the x, y, and z axes.
 *
 * @param out The output matrix where the translation matrix will be stored.
 * @param dx The translation amount along the x-axis.
 * @param dy The translation amount along the y-axis.
 * @param dz The translation amount along the z-axis.
 */
int M3d_make_translation(double out[4][4], double dx, double dy, double dz);

/**
 * Creates a scaling matrix.
 *
 * This function creates a 4x4 scaling matrix with the specified scaling factors
 * along the x, y, and z axes.
 *
 * @param out The output scaling matrix.
 * @param sx The scaling factor along the x-axis.
 * @param sy The scaling factor along the y-axis.
 * @param sz The scaling factor along the z-axis.
 */
int M3d_make_scaling(double out[4][4], double sx, double sy, double sz);

/**
 * Makes a 4x4 rotation matrix around the x-axis using a given cosine and sine values.
 *
 * @param out The output 4x4 matrix.
 * @param cs The cosine value.
 * @param sn The sine value.
 */
int M3d_make_x_rotation_cs(double out[4][4], double cs, double sn);

/**
 * Makes a 4x4 rotation matrix around the y-axis using the given cosine and sine values.
 *
 * @param out The output matrix where the result will be stored.
 * @param cs The cosine value of the rotation angle.
 * @param sn The sine value of the rotation angle.
 */
int M3d_make_y_rotation_cs(double out[4][4], double cs, double sn);


/**
 * Makes a 4x4 rotation matrix around the z-axis using the provided cosine and sine values.
 *
 * @param out The output matrix where the result will be stored.
 * @param cs The cosine value of the rotation angle.
 * @param sn The sine value of the rotation angle.
 */
int M3d_make_z_rotation_cs(double out[4][4], double cs, double sn);


/**
 * Multiplies two 4x4 matrices and stores the result in a third matrix.
 * 
 * this is SAFE, i.e. the user can make a call such as
 * `M3d_mat_mult(p,  p,q)` or `M3d_mat_mult(p,  q,p)` or  `M3d_mat_mult(p, p,p)`
 *
 * @param res The resulting matrix after multiplication.
 * @param a The first matrix to be multiplied.
 * @param b The second matrix to be multiplied.
 */
int M3d_mat_mult(double res[4][4], double a[4][4], double b[4][4]);

/**
 * Multiplies a 3D point by a 4x4 matrix.
 *
 * This function multiplies a 3D point Q by a 4x4 matrix m and stores the result in P. SAFE, user may make a call like M3d_mat_mult_pt (W, m,W) ;
 *
 * @param P The resulting 3D point after multiplication.
 * @param m The 4x4 matrix.
 * @param Q The input 3D point.
 */
int M3d_mat_mult_pt(double P[3], double m[4][4], double Q[3]);

/**
 * Multiplies a matrix by a set of points.
 *
 * This function multiplies a 4x4 matrix by a set of points in 3D space.
 * The resulting transformed points are stored in the arrays X[], Y[], and Z[].
 * SAFE, user may make a call like `M3d_mat_mult_points (x,y,z,  m, x,y,z,  n)`
 *
 * @param X[]        The array to store the transformed x-coordinates.
 * @param Y[]        The array to store the transformed y-coordinates.
 * @param Z[]        The array to store the transformed z-coordinates.
 * @param m[4][4]    The 4x4 matrix to multiply the points by.
 * @param x[]        The array of x-coordinates of the points.
 * @param y[]        The array of y-coordinates of the points.
 * @param z[]        The array of z-coordinates of the points.
 * @param numpoints  The number of points to transform.
 */
int M3d_mat_mult_points(double X[], double Y[], double Z[],
						double m[4][4],
						double x[], double y[], double z[], int numpoints);

/**
 * Calculates the cross product of two 3D vectors.
 *
 * @param res The resulting cross product vector.
 * @param a The first input vector.
 * @param b The second input vector.
 */
int M3d_x_product(double res[3], double a[3], double b[3]);

/**
 * Creates a movement sequence matrix and its inverted version.
 *
 * This function generates a 4x4 movement sequence matrix and its inverted version based on the given parameters.
 *
 * @param out The output movement sequence matrix.
 * @param out_inverted The output inverted movement sequence matrix.
 * @param n The number of movements in the sequence.
 * @param movement_type An array of movement types.
 * @param mparam An array of movement parameters.
 */
void M3d_make_movement_sequence_matrix(double out[4][4], double out_inverted[4][4], int n, int movement_type[], double mparam[]);

void M3d_view(double v[4][4], double vi[4][4],  double eyeA[3], double coiA[3], double upA[3]);

#endif