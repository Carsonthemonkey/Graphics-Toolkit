#ifndef LIGHTMODEL_H
#define LIGHTMODEL_H

#include "vector.h"
#include "colors.h"
#include "camera.h"
#include "texture.h"

extern const double AMBIENT;

/**
 * @brief A structure representing a point light for use with the Phong lightmodel
 */
typedef struct PhongLight {
    Vector3 position;
    Color3 diffuse;
    Color3 specular;
} PhongLight;

/**
 * @brief A structure representing a material to be used with the Phong lightmodel
 */
typedef struct PhongMaterial {
    Color3 base_color;
    Color3 diffuse;
    Color3 specular;
    double shininess;
    Texture texture_diffuse;
    Texture texture_displacement;
    double displacement_scale;
    Texture texture_specular;
} PhongMaterial;

/**
 * @brief Calculates the lighting of a given point based on the Phong lightmodel
 * 
 * @param position The world space position of the point
 * @param normal The world space normal vector of the point
 * @param camera The camera to calculate the lightmodel from
 * @param lights An array of PhongLights in world space to use for lighting calculations
 * @param num_lights The number of PhongLights in the array
 * @return Color3 The color of the point based on the phonglighting
 */
Color3 phong_lighting(Vector3 position, Vector3 normal, Camera cam, PhongMaterial material, PhongLight* lights, int num_lights);

Color3 phong_lighting_eye(Vector3 position, Vector3 normal, Vector3 eye, PhongMaterial material, PhongLight* lights, int num_lights);

/**
 * @brief Draws a gizmo to display point lights in the scene
 * 
 * @param light The light to draw
 * @param cam The camera to draw the light from
 * @param width The width of the window
 * @param height The height of the window
 */
void draw_light(PhongLight light, Camera cam, int width, int height);

/**
 * @brief Draws multiple lights in the scene
 * 
 * @param lights An array of lights to draw
 * @param cam The camera to draw the lights from
 * @param width The width of the window
 * @param height The height of the window
 */
void draw_lights(PhongLight* lights, int num_lights, Camera cam, int width, int height);

#endif