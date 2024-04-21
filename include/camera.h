#ifndef CAMERA_H
#define CAMERA_H

#include "vector.h"
#include "scope.h"
#include <stdbool.h>

/**
 * @brief A struct representing a 3D camera. 
 * 
 * The camera's transform should not be modified directly. Instead use transform_camera, rotate_camera, translate_camera
 */
typedef struct Camera {
    Vector3 eye;
    Vector3 coi;
    Vector3 up;
    double half_fov_degrees;
    double focal_length;
    double depth_of_field;
    
    double near_clip_plane;
    double far_clip_plane;
    double view_matrix[4][4];
    double inverse_view_matrix[4][4];
} Camera;

/**
 * Calculates the view matrix and its inverted form for a given camera.
 *
 * @param out The output view matrix.
 * @param out_inverted The output inverted view matrix. Can be NULL
 * @param cam The camera object.
 */
void make_camera_view_matrix(double out[4][4], double out_inverted[4][4], Camera cam);

/**
 * @brief Transforms a 3D point from world space to camera space
 * 
 * @param point The world space point to be transformed
 * @param cam The camera used for the transformation
 */
Vector3 to_camera_space(Vector3 point, Camera cam);

/**
 * Transforms a 3D point in camera space to screen space.
 *
 * @param point The 3D point to transform.
 * @param cam The camera used for the transformation.
 * @return The transformed point in camera screen space as a 2D vector.
 */
Vector2 to_camera_screen_space(Vector3 point, Camera cam);

/**
 * Converts a point in camera coordinates to pixel coordinates.
 *
 * @param camera_point The point in camera coordinates.
 * @param width The width of the screen in pixels.
 * @param height The height of the screen in pixels.
 * @return The point in pixel coordinates.
 */
Vector2 to_window_coordinates(Vector2 camera_point, double width, double height);

/**
 * @brief Transforms a point from global space to screen space pixel coordinates
 * 
 * @param out A Vector2 that will be set to the pixel coordinates of the point
 * @param global_point The point in global space
 * @param cam The camera used for the transformation
 * @param screen_width The width of the screen in pixels
 * @param screen_height The height of the screen in pixels
 * @return true if the point is visible to the camera
 * @return false if the point is not visible to the camera
 */
bool point_to_window(Vector2* out, Vector3 global_point, Camera cam, double screen_width, double screen_height);

/**
 * @brief Transforms the camera based on the provided transform matrix
 * 
 * @param cam A pointer to the Camera to be transformed
 * @param transform The transform to apply to the camera
 */
void transform_camera(Camera* cam, double transform[4][4]);

/**
 * @brief Translates the camera by the specified translation vector.
 *
 * @param cam A pointer to the camera to be translated
 * @param translation The translation vector.
 * @param scope The coordinate system to rotate in (global or local)
 */
void translate_camera(Camera* cam, Vector3 translation,  enum TransformScope scope);

/**
 * @brief Rotates the camera on its origin around the x axis
 * 
 * @param cam A pointer to the camera to be rotated
 * @param x_degrees Degrees to be rotated on the x axis
 * @param scope The coordinate system to rotate in (global or local)
 */
void rotate_camera_x_degrees(Camera* cam, double x_degrees,  enum TransformScope scope);

/**
 * @brief Rotates the camera on its origin around the Y axis
 * 
 * @param cam A pointer to the camera to be rotated
 * @param x_degrees Degrees to be rotated on the y axis
 * @param scope The coordinate system to rotate in (global or local)
 */
void rotate_camera_y_degrees(Camera* cam, double y_degrees,  enum TransformScope scope);

/**
 * @brief Rotates the camera on its origin around the Z axis
 * 
 * @param cam A pointer to the camera to be rotated
 * @param x_degrees Degrees to be rotated on the z axis
 * @param scope The coordinate system to rotate in (global or local)
 */
void rotate_camera_z_degrees(Camera* cam, double z_degrees,  enum TransformScope scope);

/**
 * Determines if a camera space point is visible to the camera.
 *
 * @param cam The camera object.
 * @param point The point to check for visibility. It must be in camera space (view matrix already applied)
 * @return true if the point is visible to the camera, false otherwise.
 */
bool is_visible_to_camera(Camera cam, Vector3 point);



#endif
