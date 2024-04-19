#ifndef PATH_TRACE_H
#define PATH_TRACE_H

#include <stdbool.h>
#include "vector.h"
#include "camera.h"
#include "colors.h"
#include "mesh.h"


typedef struct {
    int width, height;
    Camera* main_camera;
    int num_meshes;
    Mesh* meshes;
} PathTracedScene;

typedef struct {
    Vector3 origin;
    Vector3 direction;
} Ray;

/**
 * @brief Finds the intersection of a ray with a given triangle
 * 
 * @param location_out A vector to write the intersection location to
 * @param ray The ray to be intersected
 * @param triangle The triangle with which to attempt the intersection
 * @return true The ray does intersect the triangle
 * @return false The ray does not intersect the triangle
 */
bool intersect_triangle(double* t_out, double closest_t, Ray ray, Triangle triangle);

/**
 * @brief Draws an entire path traced scene
 * 
 * @param scene The scene to be drawn to the screen
 */
void path_trace_scene(PathTracedScene scene);

#endif