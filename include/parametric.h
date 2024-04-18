/**
 * @file scene.h
 * @author Carson Reader
 * @brief Functions and structs to manage and draw objects in the scene. Handles z-buffer and different view modes
 * @version 0.1
 * @date 2024-02-19
 */
#ifndef PARAMETRIC_H
#define PARAMETRIC_H

#include "FPToolkit.h"
#include "vector.h"
#include "camera.h"
#include "lightmodel.h"

enum ViewMode {
    LIT,
    UNLIT,
    Z_BUFF,
    NORMAL,
    UV
};

typedef struct {
    Vector3 (*f)(double, double);
    double u_start;
    double u_end;
    double u_step;
    double v_start;
    double v_end;
    double v_step;
    double transform[4][4];
    PhongMaterial material;
} ParametricObject3D;

/**
 * @brief Draws a parametric 3D object in the scene
 * 
 * @param object The Parametric object to draw
 * @param cam The camera to use for the drawing
 * @param lights An array of lights that will be applied to the object
 * @param num_lights The number of lights
 * @param width The width of the screen
 * @param height The height of the screen
 * @param z_buffer The z_buffer to be used to draw the object
 * @param mode the view mode to draw the objects in
 */
void draw_parametric_object_3d(ParametricObject3D object, Camera cam, PhongLight* lights, int num_lights, int width, int height, double z_buffer[width][height], enum ViewMode mode);

/**
 * @brief Draws multiple parametric objects
 * 
 * @param objects An array of parametric objects to draw
 * @param num_objs The number of parametric objects in the array
 * @param cam 
 * @param lights An array of lights that will be applied to the object
 * @param num_lights The number of lights
 * @param width The width of the screen
 * @param height The height of the screen
 * @param z_buffer The z_buffer to be used to draw the object
 * @param mode the view mode to draw the objects in
 */
void draw_parametric_objects_3d(ParametricObject3D* objects, int num_objs, Camera cam, PhongLight* lights, int num_lights, int width, int height, double z_buffer[width][height], enum ViewMode mode);

/**
 * @brief Parametric function for a sphere
 * 
 * @param u The u parameter (0 to 2π for full sphere)
 * @param v The v parameter (0 to π for full sphere)
 * @return Vector3 The resulting point on the sphere
 */
Vector3 param_sphere(double u, double v);

/**
 * @brief Parametric function for a plane
 * 
 * 
 * @param u The u parameter
 * @param v The v parameter
 * @return Vector3 The resulting point on the plane
 * 
 */
Vector3 param_plane(double u, double v);

/**
 * @brief Parametric function for a plane
 * 
 * @param u The u parameter (0 to 2π for full cylinder)
 * @param v The v parameter
 * @return Vector3 The resulting point on the cylinder
 */
Vector3 param_cylinder(double u, double v);

extern double TORUS_MINOR_RADIUS;
extern double TORUS_MAJOR_RADIUS;
/**
 * @brief Parametric function for a torus
 * 
 * @param u The u parameter (0 to 2π for full torus)
 * @param v The v parameter (0 to 2π for full torus)
 * @return Vector3 
 */
Vector3 param_torus(double u, double v);


/**
 * @brief Parametric function for a twisted torus
 * 
 * @param u The u parameter (0 to 2π for full torus)
 * @param v The v parameter (0 to 2π for full torus)
 * @return Vector3 
 */
Vector3 param_twisted_torus(double u, double v);

/**
 * @brief Parametric function for a square torus
 * 
 * @param u The u parameter (0 to 2π for full torus)
 * @param v The v parameter (0 to 2π for full torus)
 * @return Vector3 
 */
Vector3 param_square_torus(double u, double v);


#endif