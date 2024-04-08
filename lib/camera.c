#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include "camera.h"
#include "scope.h"
#include "matrix.h"
#include "vector.h"
#include "trig.h"
#include "math.h"
#include "M3d_matrix_tools.h"

void make_camera_view_matrix(double out[4][4], double out_inverted[4][4], Camera cam){
    // This is hacky, probably good to rewrite this function to be better later
    M3d_view(out, out_inverted, (double *)&cam.eye, (double *)&cam.coi, (double *)&cam.up);
}

Vector3 to_camera_space(Vector3 point, Camera cam){
    return mat4_mult_point(point, cam.view_matrix);
}

//TODO: make this work for non square aspect ratios
Vector2 to_camera_screen_space(Vector3 point, Camera cam){    
    Vector2 screen_point;
    double film_distance = 1 / tan(to_radians(cam.half_fov_degrees));
    screen_point.x = point.x / point.z * film_distance;
    screen_point.y = point.y / point.z * film_distance;
    return screen_point;
}

Vector2 to_window_coordinates(Vector2 point, double width, double height){
    Vector2 window_coord;
    window_coord.x = (point.x * width / 2) + width / 2;
    window_coord.y = (point.y * height / 2) + height / 2;
    return window_coord;
}

//TODO: make this work with z-buffer
bool point_to_window(Vector2* out, Vector3 global_point, Camera cam, double screen_width, double screen_height){
    Vector3 camera_point = to_camera_space(global_point, cam);
    if(!is_visible_to_camera(cam, camera_point)) return false;
    Vector2 screen_point = to_window_coordinates(to_camera_screen_space(camera_point, cam), screen_width, screen_height);
    out->x = screen_point.x;
    out->y = screen_point.y;
    return true;
}

void transform_camera(Camera* cam, double transform[4][4]){
    cam->eye = mat4_mult_point(cam->eye, transform);
    cam->coi = mat4_mult_point(cam->coi, transform);
    cam->up  = mat4_mult_point(cam->up, transform);

    //Update the camera's view matrix
    make_camera_view_matrix(cam->view_matrix, cam->inverse_view_matrix, *cam);
}

void translate_camera(Camera* cam, Vector3 translation, enum TransformScope scope){
    double transform[4][4] = IDENTITY_INITIALIZER;
    if(scope == LOCAL) M3d_mat_mult(transform, cam->view_matrix, transform);
    double trans_mat[4][4];
    M3d_make_translation(trans_mat, translation.x, translation.y, translation.z);
    M3d_mat_mult(transform, trans_mat, transform);
    if(scope == LOCAL) M3d_mat_mult(transform, cam->inverse_view_matrix, transform);
    transform_camera(cam, transform);
}

void rotate_camera_x_degrees(Camera* cam, double x_degrees, enum TransformScope scope){
    double transform[4][4] = IDENTITY_INITIALIZER;

    if(scope == LOCAL) M3d_mat_mult(transform, cam->view_matrix, transform);
    else M3d_make_translation(transform, -cam->eye.x, -cam->eye.y, -cam->eye.z);

    double rads = to_radians(x_degrees);
    double temp_mat[4][4];

    M3d_make_x_rotation_cs(temp_mat, cos(rads), sin(rads));
    M3d_mat_mult(transform, temp_mat, transform);
    
    if(scope == LOCAL) M3d_mat_mult(transform, cam->inverse_view_matrix, transform);
    transform_camera(cam, transform);
}

void rotate_camera_y_degrees(Camera* cam, double y_degrees, enum TransformScope scope){
    double transform[4][4] = IDENTITY_INITIALIZER;

    if(scope == LOCAL) M3d_mat_mult(transform, cam->view_matrix, transform);
    else M3d_make_translation(transform, -cam->eye.x, -cam->eye.y, -cam->eye.z);

    double rads = to_radians(y_degrees);
    double temp_mat[4][4];

    M3d_make_y_rotation_cs(temp_mat, cos(rads), sin(rads));
    M3d_mat_mult(transform, temp_mat, transform);
    
    if(scope == LOCAL) M3d_mat_mult(transform, cam->inverse_view_matrix, transform);
    transform_camera(cam, transform);
}

void rotate_camera_z_degrees(Camera* cam, double z_degrees, enum TransformScope scope){
    double transform[4][4] = IDENTITY_INITIALIZER;

    if(scope == LOCAL) M3d_mat_mult(transform, cam->view_matrix, transform);
    else M3d_make_translation(transform, -cam->eye.x, -cam->eye.y, -cam->eye.z);

    double rads = to_radians(z_degrees);
    double temp_mat[4][4];

    M3d_make_z_rotation_cs(temp_mat, cos(rads), sin(rads));
    M3d_mat_mult(transform, temp_mat, transform);
    
    if(scope == LOCAL) M3d_mat_mult(transform, cam->inverse_view_matrix, transform);
    transform_camera(cam, transform);
}

bool is_visible_to_camera(Camera cam, Vector3 point){
    if(point.z < cam.near_clip_plane || point.z > cam.far_clip_plane){return false;}
    double half_angle_rads = to_radians(cam.half_fov_degrees);

    //TODO: make this work for non square aspect ratios
    double extent = point.z * tan(half_angle_rads);
    if(fabs(point.x) > extent){return false;}
    if(fabs(point.y) > extent){return false;}
    return true;
}

//TODO: make this work for non square aspect ratios
double get_film_distance(Camera cam){
    return 1 / tan(to_radians(cam.half_fov_degrees));
}