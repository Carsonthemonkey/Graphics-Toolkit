#include <math.h>
#include <stdio.h>
#ifndef M3D_MATRIX_TOOLS
#define M3D_MATRIX_TOOLS

/* More intuitive indices for vectors */
#define Xi 0
#define Yi 1
#define Zi 2

/*

 ( x')          (x)
 ( y')  =   M * (y)
 ( z')          (z)
 ( 1 )          (1)

instead of (x',y',z',1) = (x,y,z,1) * M

*/

/**
 * Prints the elements of a 4x4 matrix.
 *
 * @param a The matrix to be printed.
 */
int M3d_print_mat(double a[4][4]) {
  int r, c;
  for (r = 0; r < 4; r++) {
    for (c = 0; c < 4; c++) {
      printf(" %12.4lf ", a[r][c]);
    }
    printf("\n");
  }

  return 1;
}

/**
 * Copies the contents of a 4x4 matrix into another 4x4 matrix.
 *
 * @param out The destination matrix where the contents will be copied to.
 * @param matrix The source matrix from which the contents will be copied.
 */
int M3d_copy_mat(double out[4][4], double matrix[4][4])
// a = b
{
  int r, c;
  for (r = 0; r < 4; r++) {
    for (c = 0; c < 4; c++) {
      out[r][c] = matrix[r][c];
    }
  }

  return 1;
}

/**
 * @brief Sets the given 4x4 matrix to the identity matrix.
 *
 * This function modifies the input matrix by setting all elements to 0, except
 * for the diagonal elements which are set to 1.
 *
 * @param a The 4x4 matrix to be set to the identity matrix.
 */
int M3d_make_identity(double a[4][4])
// a = I
{
  int r, c;
  for (r = 0; r < 4; r++) {
    for (c = 0; c < 4; c++) {
      if (r == c)
        a[r][c] = 1.0;
      else
        a[r][c] = 0.0;
    }
  }

  return 1;
}

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
int M3d_make_translation(double out[4][4], double dx, double dy, double dz) {
  M3d_make_identity(out);
  out[0][3] = dx;
  out[1][3] = dy;
  out[2][3] = dz;
  return 1;
}

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
int M3d_make_scaling(double out[4][4], double sx, double sy, double sz) {
  M3d_make_identity(out);
  out[0][0] = sx;
  out[1][1] = sy;
  out[2][2] = sz;
  return 1;
}

/**
 * Makes a 4x4 rotation matrix around the x-axis using a given cosine and sine
 * values.
 *
 * @param out The output 4x4 matrix.
 * @param cs The cosine value.
 * @param sn The sine value.
 */
int M3d_make_x_rotation_cs(double out[4][4], double cs, double sn)
// this one assumes cosine and sine are already known
{
  M3d_make_identity(out);

  out[1][1] = cs;
  out[1][2] = -sn;
  out[2][1] = sn;
  out[2][2] = cs;

  return 1;
}

/**
 * Makes a 4x4 rotation matrix around the y-axis using the given cosine and sine
 * values.
 *
 * @param out The output matrix where the result will be stored.
 * @param cs The cosine value of the rotation angle.
 * @param sn The sine value of the rotation angle.
 */
int M3d_make_y_rotation_cs(double out[4][4], double cs, double sn)
// this one assumes cosine and sine are already known
{
  M3d_make_identity(out);

  out[0][0] = cs;
  out[0][2] = sn;
  out[2][0] = -sn;
  out[2][2] = cs;

  return 1;
}

/**
 * Makes a 4x4 rotation matrix around the z-axis using the provided cosine and
 * sine values.
 *
 * @param out The output matrix where the result will be stored.
 * @param cs The cosine value of the rotation angle.
 * @param sn The sine value of the rotation angle.
 */
int M3d_make_z_rotation_cs(double out[4][4], double cs, double sn)
// this one assumes cosine and sine are already known
{
  M3d_make_identity(out);

  out[0][0] = cs;
  out[0][1] = -sn;
  out[1][0] = sn;
  out[1][1] = cs;

  return 1;
}

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
int M3d_mat_mult(double res[4][4], double a[4][4], double b[4][4])
// res = a * b
// this is SAFE, i.e. the user can make a call such as
// M3d_mat_mult(p,  p,q) or M3d_mat_mult(p,  q,p) or  M3d_mat_mult(p, p,p)
{
  double sum;
  int k;
  int r, c;
  double tmp[4][4];

  for (r = 0; r < 4; r++) {
    for (c = 0; c < 4; c++) {
      sum = 0.0;
      for (k = 0; k < 4; k++) {
        sum = sum + a[r][k] * b[k][c];
      }
      tmp[r][c] = sum;
    }
  }

  M3d_copy_mat(res, tmp);

  return 1;
}

/**
 * Multiplies a 3D point by a 4x4 matrix.
 *
 * This function multiplies a 3D point Q by a 4x4 matrix m and stores the result
 * in P. SAFE, user may make a call like M3d_mat_mult_pt (W, m,W) ;
 *
 * @param P The resulting 3D point after multiplication.
 * @param m The 4x4 matrix.
 * @param Q The input 3D point.
 */
int M3d_mat_mult_pt(double P[3], double m[4][4], double Q[3])
// P = m*Q
// SAFE, user may make a call like M3d_mat_mult_pt (W, m,W) ;
{
  double u, v, t;

  u = m[0][0] * Q[0] + m[0][1] * Q[1] + m[0][2] * Q[2] + m[0][3];
  v = m[1][0] * Q[0] + m[1][1] * Q[1] + m[1][2] * Q[2] + m[1][3];
  t = m[2][0] * Q[0] + m[2][1] * Q[1] + m[2][2] * Q[2] + m[2][3];

  P[0] = u;
  P[1] = v;
  P[2] = t;

  return 1;
}

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
int M3d_mat_mult_points(double X[], double Y[], double Z[], double m[4][4],
                        double x[], double y[], double z[], int numpoints)
// |X0 X1 X2 ...|       |x0 x1 x2 ...|
// |Y0 Y1 Y2 ...| = m * |y0 y1 y2 ...|
// |Z0 Z1 Z2 ...|       |z0 z1 z2 ...|
// | 1  1  1 ...|       | 1  1  1 ...|

// SAFE, user may make a call like M3d_mat_mult_points (x,y,z,  m, x,y,z,  n) ;
{
  double u, v, t;
  int i;

  for (i = 0; i < numpoints; i++) {
    u = m[0][0] * x[i] + m[0][1] * y[i] + m[0][2] * z[i] + m[0][3];
    v = m[1][0] * x[i] + m[1][1] * y[i] + m[1][2] * z[i] + m[1][3];
    t = m[2][0] * x[i] + m[2][1] * y[i] + m[2][2] * z[i] + m[2][3];

    X[i] = u;
    Y[i] = v;
    Z[i] = t;
  }
  return 1;
}

/**
 * Calculates the cross product of two 3D vectors.
 *
 * @param res The resulting cross product vector.
 * @param a The first input vector.
 * @param b The second input vector.
 */
int M3d_x_product(double res[3], double a[3], double b[3])
// res = a x b  , cross product of two vectors
// SAFE: it is ok to make a call such as
// D3d_x_product (a,  a,b) or
// D3d_x_product (b,  a,b) or
// D3d_x_product (a,  a,a)
{
  double r[3];
  int v;

  r[0] = a[1] * b[2] - b[1] * a[2];  // a.y * b.z - b.y * a.z
  r[1] = b[0] * a[2] - a[0] * b[2];  // a.x * b.z - b.x * a.z
  r[2] = a[0] * b[1] - b[0] * a[1];  // a.x * b.y - b.x * a.y

  res[0] = r[0];
  res[1] = r[1];
  res[2] = r[2];

  if ((res[0] == 0) && (res[1] == 0) && (res[2] == 0)) {
    v = 0;
  } else {
    v = 1;
  }

  return v;
}

//===========================================================================
// For Advanced Graphics :
//===========================================================================

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
 * Creates a movement sequence matrix and its inverted version.
 *
 * This function generates a 4x4 movement sequence matrix and its inverted
 * version based on the given parameters.
 *
 * @param out The output movement sequence matrix.
 * @param out_inverted The output inverted movement sequence matrix.
 * @param n The number of movements in the sequence.
 * @param movement_type An array of movement types.
 * @param mparam An array of movement parameters.
 */
void M3d_make_movement_sequence_matrix(double out[4][4],
                                       double out_inverted[4][4], int n,
                                       int movement_type[], double mparam[]) {
  double result[4][4];
  double inverse_result[4][4];
  M3d_make_identity(result);
  M3d_make_identity(inverse_result);

  for (int i = 0; i < n; i++) {
    double mat[4][4];
    double inverse_mat[4][4];
    M3d_make_identity(mat);
    M3d_make_identity(inverse_mat);
    int movement = movement_type[i];
    double rads;
    switch (movement) {
      // Translation
      case TX:
        M3d_make_translation(mat, mparam[i], 0, 0);
        M3d_make_translation(inverse_mat, -mparam[i], 0, 0);
        break;

      case TY:
        M3d_make_translation(mat, 0, mparam[i], 0);
        M3d_make_translation(inverse_mat, 0, -mparam[i], 0);
        break;

      case TZ:
        M3d_make_translation(mat, 0, 0, mparam[i]);
        M3d_make_translation(inverse_mat, 0, 0, -mparam[i]);
        break;

      // Rotation
      // apparently only sin needs to be negated for rotation
      case RX:
        rads = mparam[i] * (M_PI / 180);
        M3d_make_x_rotation_cs(mat, cos(rads), sin(rads));
        M3d_make_x_rotation_cs(inverse_mat, cos(rads), -sin(rads));
        break;

      case RY:
        rads = mparam[i] * (M_PI / 180);
        M3d_make_y_rotation_cs(mat, cos(rads), sin(rads));
        M3d_make_y_rotation_cs(inverse_mat, cos(rads), -sin(rads));
        break;

      case RZ:
        rads = mparam[i] * (M_PI / 180);
        M3d_make_z_rotation_cs(mat, cos(rads), sin(rads));
        M3d_make_z_rotation_cs(inverse_mat, cos(rads), -sin(rads));
        break;

      // Scale
      case SX:
        M3d_make_scaling(mat, mparam[i], 1, 1);
        M3d_make_scaling(inverse_mat, 1 / mparam[i], 1, 1);
        break;

      case SY:
        M3d_make_scaling(mat, 1, mparam[i], 1);
        M3d_make_scaling(inverse_mat, 1, 1 / mparam[i], 1);
        break;

      case SZ:
        M3d_make_scaling(mat, 1, 1, mparam[i]);
        M3d_make_scaling(inverse_mat, 1, 1, 1 / mparam[i]);
        break;

      // Negation
      case NX:
        M3d_make_scaling(mat, -1, 1, 1);
        M3d_make_scaling(inverse_mat, -1, 1, 1);
        break;

      case NY:
        M3d_make_scaling(mat, 1, -1, 1);
        M3d_make_scaling(inverse_mat, 1, -1, 1);
        break;

      case NZ:
        M3d_make_scaling(mat, 1, 1, -1);
        M3d_make_scaling(inverse_mat, 1, 1, -1);
        break;

      default:
        break;
    }

    M3d_mat_mult(result, mat, result);
    M3d_mat_mult(inverse_result, inverse_result, inverse_mat);
  }
  M3d_copy_mat(out, result);
  if (out_inverted != NULL) {
    M3d_copy_mat(out_inverted, inverse_result);
  }
}

double magnitude(double v[3]) {
  return sqrt(pow(v[Xi], 2) + pow(v[Yi], 2) + pow(v[Zi], 2));
}

void M3d_view(double v[4][4], double vi[4][4], double eyeA[3], double coiA[3],
              double upA[3]) {
  double mat[4][4];
  double mat_inverted[4][4];
  double temp_mat[4][4];
  double new_coi[3];

  // Translate to origin
  M3d_make_translation(mat, -eyeA[0], -eyeA[1], -eyeA[2]);
  M3d_make_translation(mat_inverted, eyeA[0], eyeA[1], eyeA[2]);
  M3d_mat_mult_pt(new_coi, mat, coiA);

  // Rotate to be aligned with Y, Z plane
  double hypotonuse = sqrt(pow(new_coi[Xi], 2) + pow(new_coi[Zi], 2));
  double sn = new_coi[Xi] / hypotonuse;
  double cs = new_coi[Zi] / hypotonuse;

  M3d_make_y_rotation_cs(
      temp_mat, cs,
      -sn);  //? Don't really understand why sin is negative for this one only
  M3d_mat_mult(mat, temp_mat, mat);

  // inverse
  M3d_make_y_rotation_cs(temp_mat, cs, sn);
  M3d_mat_mult(mat_inverted, mat_inverted, temp_mat);  // opposite order

  // Rotate to that coi is aligned along positive Z axis (rotation on Y, Z
  // plane)
  M3d_mat_mult_pt(new_coi, mat, coiA);  // move coi into Y, Z plane

  hypotonuse = sqrt(pow(new_coi[Zi], 2) + pow(new_coi[Yi], 2));
  sn = new_coi[Yi] / hypotonuse;
  cs = new_coi[Zi] / hypotonuse;

  M3d_make_x_rotation_cs(temp_mat, cs, sn);
  M3d_mat_mult(mat, temp_mat, mat);

  // inverse
  M3d_make_x_rotation_cs(temp_mat, cs, -sn);
  M3d_mat_mult(mat_inverted, mat_inverted, temp_mat);

  M3d_mat_mult_pt(new_coi, mat, coiA);

  // Rotate so that up is aligned with positive Y axis
  double new_up[3];
  M3d_mat_mult_pt(new_up, mat, upA);

  hypotonuse = sqrt(pow(new_up[Xi], 2) + pow(new_up[Yi], 2));
  sn = new_up[Xi] / hypotonuse;
  cs = new_up[Yi] / hypotonuse;

  M3d_make_z_rotation_cs(temp_mat, cs, sn);
  M3d_mat_mult(mat, temp_mat, mat);

  // inverse
  M3d_make_z_rotation_cs(temp_mat, cs, -sn);
  M3d_mat_mult(mat_inverted, mat_inverted, temp_mat);

  M3d_copy_mat(v, mat);
  if (vi != NULL) {
    M3d_copy_mat(vi, mat_inverted);
  }
};

/* Some functions to work with vectors and matrices more easily */

#include "vector.h"

/**
 * Translates the given matrix by the specified translation vector.
 *
 * @param matrix The matrix to be translated.
 * @param translation The translation vector.
 */
void translate_matrix(double matrix[4][4], Vector3 translation) {
  double trans_mat[4][4];
  M3d_make_translation(trans_mat, translation.x, translation.y, translation.z);
  M3d_mat_mult(matrix, trans_mat, matrix);
}

/**
 * @brief Scales a 4x4 matrix by the given scale vector.
 *
 * This function multiplies each element of the matrix by the corresponding
 * element of the scale vector, resulting in a scaled matrix.
 *
 * @param matrix The 4x4 matrix to be scaled.
 * @param scale The scale vector containing the scaling factors for each axis.
 */
void scale_matrix(double matrix[4][4], Vector3 scale) {
  double scale_mat[4][4];
  M3d_make_scaling(scale_mat, scale.x, scale.y, scale.z);
  M3d_mat_mult(matrix, scale_mat, matrix);
}

// parametric functions

/**
 * @return the sign of double a
 */
int sgn(double a) {
  if (a > 0) {
    return 1;
  } else if (a < 0) {
    return -1;
  } else {
    return 0;
  }
}

/**
 * @returns vector p transformed by the given matrix
 */
Vector2 transformPoint(Vector2 p, double matrix[4][4]) {
  M3d_mat_mult_pt((double*)&p, matrix, (double*)&p);
  return p;
}

/**
 * @brief Draws a 2d parametric function using the given transformation matrix.
 * @param f The parametric function to be drawn.
 * @param transform The transformation matrix to be applied to the function.
 * @param uLow The lower bound of the parameter.
 * @param uHigh The upper bound of the parameter.
 * @param uStep The step size for the parameter.
 */

#define DRAW_NORMAL 0
int parametricDraw(Vector2 (*f)(double), double transform[4][4], double uLow,
                   double uHigh, double uStep) {
  //   for (double u = uLow; u < uHigh; u += uStep) {
  int scale = (int)((uHigh - uLow) / uStep);
  for (int i = 0; i < scale; i++) {
    printf("%d %lf %lf %lf\n", scale, uLow, uHigh, uStep);
    double u = uLow + i * uStep;
    Vector2 p = transformPoint(f(u), transform);
    if (DRAW_NORMAL &&
        (i % 10 == 0) /*&& ((int)fmod((u / uStep), 20) == 10)*/) {
      printf("u current: %lf\t%d\n", u, (int)fmod((u / uStep), 20));
      Vector2 p2 = transformPoint(f(u + 0.01), transform);
      Vector2 normal = (Vector2){p2.y - p.y, p.x - p2.x};
      double length = sqrt(normal.x * normal.x + normal.y * normal.y);
      normal.x /= length;
      normal.y /= length;
      normal.x *= 50;
      normal.y *= 50;
      Vector2 p3 = (Vector2){p.x + normal.x, p.y + normal.y};
      vec2_print(p);
      vec2_print(p2);
      vec2_print(p3);
      G_line(p.x, p.y, p3.x, p3.y);
      printf("end point\n");
    }
    G_pixel(p.x, p.y);
  }
  return 0;
}

#endif