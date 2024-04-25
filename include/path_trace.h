#ifndef PATH_TRACE_H
#define PATH_TRACE_H

#include <stdbool.h>
#include "vector.h"
#include "camera.h"
#include "colors.h"
#include "mesh.h"

typedef struct {
    Vector3 position;
    double radius;
    Color3 intensity;
} PointLight;

typedef struct {
    // image buffers
    int width, height;
    Color3* light_buffer;
    Color3f* denoise_buffer;
    Color3f* normal_buffer;
    Color3f* albedo_buffer;
    Color3f* output_buffer;

    // Camera
    Camera* main_camera;

    // Scene objects
    int num_meshes;
    Mesh* meshes;
    int num_lights;
    PointLight* lights; //TODO: make this work nice with different light types

    // Post processing
    bool denoise;
    double exposure;
    Color3f (*color_transform)(Color3f color);
    Color3f (*tonemap)(Color3f color);
} PathTracedScene;

typedef struct {
    Vector3 origin;
    Vector3 direction;
} Ray;

// Maybe this should return the specific triangle later but for now this is good
typedef struct {
    double distance;
    Vector3 normal;
    Mesh intersected_mesh;
} RayHitInfo;

/**
 * @brief Initializes buffers and denoiser for a scene
 * 
 * @param scene The scene to initialize
 */
void init_scene(PathTracedScene* scene);


/**
 * @brief Frees all resources allocated for a scene
 * 
 * @param scene The scene to cleanup
 */
void cleanup_scene(PathTracedScene* scene);
/**
 * @brief Finds the intersection of a ray with a given triangle
 * 
 * @param t_out A pointer to save the distance of the triangle intersections
 * @param barycentric_out A pointer to save the barycentric coordinates of the triangle intersection
 * @param ray The ray to be intersected
 * @param triangle The triangle with which to attempt the intersection
 * @return true The ray does intersect the triangle
 * @return false The ray does not intersect the triangle
 */
bool intersect_triangle(double* t_out, Vector2* barycentric_out, double closest_t, Ray ray, Triangle triangle);

/**
 * @brief Draws an entire path traced scene
 * 
 * @param scene The scene to be drawn to the screen
 */
void path_trace_scene(PathTracedScene scene, int y_start, int y_end);

/**
 * @brief Draws the screen buffer to the screen
 * 
 * @param scene The scene whose screen buffer will be drawn
 */
void draw_screen_buffer(PathTracedScene scene);

/**
 * @brief Clears the screen buffer to a given color
 * 
 * @param light_buffer The buffer to clear
 * @param color The color to set it to
 * @param width The width of the buffer
 * @param height The height of the buffer
 */
void clear_screen_buffer(Color3* light_buffer, Color3 color, int width, int height);

/**
 * @brief Draws an entire path traced scene using multiple threads
 * 
 * @param scene The scene to be drawn to the screen
 */
void path_trace_scene_multithreaded(PathTracedScene scene);

/**
 * @brief Draws a point light gizmo in the scene
 * 
 * @param light The light to be drawn
 * @param cam The camera to draw the light from
 * @param width the 
 */
void debug_draw_light(PointLight light, Camera cam, double width, double height);



#endif