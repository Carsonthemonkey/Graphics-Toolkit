#include <stdio.h>
#include <math.h>
#include "path_trace.h"
#include "FPToolkit.h"
#include "trig.h"
#include "camera.h"
#include "matrix.h"
#include "FPToolkit.h"
#include "colors.h"
#include "vector.h"
#include "mesh.h"
#include "random.h"

const double EPSILON = 0.000001;
const int NUM_SHADOW_RAYS = 32;

bool intersect_triangle(double* t_out, double closest_t, Ray ray, Triangle triangle){
    //TODO: precompute triangle normal
    //I don't really understand this intuitively. It would be a good idea to go back to this to get a better grasp on it
    Vector3 edge_1 = vec3_sub(triangle.b->position, triangle.a->position);
    Vector3 edge_2 = vec3_sub(triangle.c->position, triangle.a->position);

    // Vector3 normal = vec3_cross_prod(edge_1, edge_2); //This may need to be normalized
    Vector3 normal = triangle.normal;
    Vector3 pvec = vec3_cross_prod(ray.direction, edge_2);
    double determinant = vec3_dot_prod(edge_1, pvec);
    if(fabs(determinant) < EPSILON) return false; //Ray is paralell to triangle
    
    double inverse_determinant = 1.0 / determinant;
    Vector3 vertex_to_origin = vec3_sub(ray.origin, triangle.a->position);

    double u = vec3_dot_prod(vertex_to_origin, pvec) * inverse_determinant;
    if(u < 0 || u > 1) return false;

    Vector3 edge_1_cross_prod = vec3_cross_prod(vertex_to_origin, edge_1);
    double v = vec3_dot_prod(ray.direction, edge_1_cross_prod) * inverse_determinant;

    if(v < 0 || u + v > 1) return false;

    double t = vec3_dot_prod(edge_2, edge_1_cross_prod) * inverse_determinant;
    if(t < closest_t && t > EPSILON) {
        if(t_out != NULL) *t_out = t;
        return true;
    }
    return false;
}

bool intersects_bounding_box(Mesh mesh, Ray ray){
    Vector3 box_min = mesh.bounding_box_min;
    Vector3 box_max = mesh.bounding_box_max;

    Vector3 direction_inv = {1.0 / ray.direction.x, 1.0 / ray.direction.y, 1.0 / ray.direction.z};
    Vector3 tmin = {(box_min.x - ray.origin.x) * direction_inv.x, (box_min.y - ray.origin.y) * direction_inv.y, (box_min.z - ray.origin.z) * direction_inv.z};
    Vector3 tmax = {(box_max.x - ray.origin.x) * direction_inv.x, (box_max.y - ray.origin.y) * direction_inv.y, (box_max.z - ray.origin.z) * direction_inv.z};

    if (ray.direction.x == 0) {
        tmin.x = ray.origin.x < box_min.x || ray.origin.x > box_max.x ? -INFINITY : INFINITY;
        tmax.x = ray.origin.x < box_min.x || ray.origin.x > box_max.x ? INFINITY : -INFINITY;
    }
    if (ray.direction.y == 0) {
        tmin.y = ray.origin.y < box_min.y || ray.origin.y > box_max.y ? -INFINITY : INFINITY;
        tmax.y = ray.origin.y < box_min.y || ray.origin.y > box_max.y ? INFINITY : -INFINITY;
    }
    if (ray.direction.z == 0) {
        tmin.z = ray.origin.z < box_min.z || ray.origin.z > box_max.z ? -INFINITY : INFINITY;
        tmax.z = ray.origin.z < box_min.z || ray.origin.z > box_max.z ? INFINITY : -INFINITY;
    }

    double t_enter = fmax(fmin(tmin.x, tmax.x), fmax(fmin(tmin.y, tmax.y), fmin(tmin.z, tmax.z)));
    double t_exit = fmin(fmax(tmin.x, tmax.x), fmin(fmax(tmin.y, tmax.y), fmax(tmin.z, tmax.z)));

    return t_enter <= t_exit && t_exit >= 0;
}

bool raycast(RayHitInfo* out, PathTracedScene scene, Ray ray){
    bool did_hit = false;
    double closest_t = INFINITY;
    double t;
    for(int m = 0; m < scene.num_meshes; m++){
        Mesh mesh = scene.meshes[m];
        if(!intersects_bounding_box(mesh, ray)) continue;
        for(int tr = 0; tr < mesh.num_tris; tr++){
            Triangle tri = mesh.tris[tr];
            if(intersect_triangle(&t, closest_t, ray, tri)){
                if(out == NULL) return true;
                did_hit = true;
                closest_t = t;
                out->intersected_mesh = mesh;
                out->distance = t;
                out->normal = tri.normal;
            }
        }
    }
    return did_hit;
}

Vector3 ray_hit_location(Ray ray, double distance){
    return vec3_add(ray.origin, vec3_scale(ray.direction, distance));
}

//TODO: add different kinds of light support here 
Color3 shadow_ray(PathTracedScene scene, Vector3 position){
    // This is not a clamped color
    Color3 direct_light = {0, 0, 0};
    Ray shadow = {.origin=position};
    for(int l = 0; l < scene.num_lights; l++){
        PointLight light = scene.lights[l];
        double shadow_ray_hit_percentage = 0;
        double light_distance = vec3_magnitude(vec3_sub(light.position, position));
        for (int r = 0; r < NUM_SHADOW_RAYS; r++){
            Vector3 dest = vec3_add(random_point_in_sphere(light.radius), light.position);
            shadow.direction = vec3_normalized(vec3_sub(dest, shadow.origin));

            //TODO!: optimize this. We only care if we hit any object, so we can return early when we hit something
            RayHitInfo hit;
            bool did_hit = raycast(&hit, scene, shadow);
            if(!did_hit || hit.distance > light_distance) shadow_ray_hit_percentage += 1.0 / NUM_SHADOW_RAYS;
        }
        //Scale according to inverse square
        //* I think this avoids the incident angle/lambertian diffuse or whatever it is called. Cosine weigthing? look into it
        Color3 contribution = vec3_scale(vec3_scale(light.intensity, shadow_ray_hit_percentage), 1.0 / (light_distance * light_distance));
        direct_light = vec3_add(contribution, direct_light);
    }
    return direct_light;
}

Color3 path_trace(PathTracedScene scene, Ray ray){
    Color3 black = {BLACK};
    RayHitInfo hit;
    bool did_hit = raycast(&hit, scene, ray);

    /* Environment lighting goes here basically */
    if(!did_hit) return ray.direction;

    Color3 result = hit.intersected_mesh.material.base_color;
    Vector3 hit_location = vec3_add(ray.origin, vec3_scale(ray.direction, hit.distance));

    Color3 direct_lighting = shadow_ray(scene, hit_location);
    return vec3_mult(direct_lighting, result);
    /* Shadow Rays */
    // Ray shadow_ray = {.origin=vec3_add(hit_location, vec3_scale(hit.normal, EPSILON))};
    // Color3 intensity = {0, 0, 0};
    // double shadow_ray_contribution = 1.0 / NUM_SHADOW_RAYS;
    // double contribution = 0;
    
    // for(int l = 0; l < scene.num_lights; l++){
    //     PointLight light = scene.lights[l];

    //     //TODO: put this into it's own function
    //     for(int r = 0; r < NUM_SHADOW_RAYS; r++){

    //         Vector3 dest = vec3_add(random_point_in_sphere(light.radius), light.position);
    //         double light_distance = vec3_distance(shadow_ray.origin, light.position) - EPSILON;
    //         shadow_ray.direction = vec3_normalized(vec3_sub(dest, shadow_ray.origin));
            
    //         for(int m = 0; m < scene.num_meshes; m++){
    //             for(int tr = 0; tr < scene.meshes[m].num_tris; tr++){
    //                 if(!intersects_bounding_box(scene.meshes[m], shadow_ray)) continue;
    //                 //TODO: make this work with multiple lights
    //                 if(intersect_triangle(NULL, light_distance, shadow_ray, scene.meshes[m].tris[tr])) goto next_ray;
    //             }
    //         }
    //         contribution += 1.0 / NUM_SHADOW_RAYS;
    //         next_ray:
    //     }
    // }

    // return vec3_scale(result, contribution);
}

void path_trace_scene(PathTracedScene scene){
    double dwidth = (double)scene.width;
    double dheight = (double)scene.height;
    //TODO: obligatory "make this work for non square aspect ratios"
    double film_extent = tan(to_radians(scene.main_camera->half_fov_degrees));

    for(int y = 0; y < scene.height; y++){
        for(int x = 0; x < scene.width; x++){
            Vector3 pixel_camera_space = {
                ((x - (dwidth / 2)) / dwidth) * (film_extent * 2),
                ((y - (dheight / 2)) / dheight) * (film_extent * 2),
                1
            };
            Vector3 world_space_dir = vec3_normalized(vec3_sub(mat4_mult_point(pixel_camera_space, scene.main_camera->inverse_view_matrix), scene.main_camera->eye));
            Ray ray = {
                .origin=scene.main_camera->eye,
                .direction=world_space_dir
            };
            Color3 pixel_color = path_trace(scene, ray);
            G_rgb(SPREAD_COL3(pixel_color));
            G_pixel(x, y);
        }
        G_display_image();
    }
}

void debug_draw_light(PointLight light, Camera cam, double width, double height){
    Vector2 light_center;
    if(!point_to_window(&light_center, light.position, cam, width, height)) return;
    G_rgb(SPREAD_VEC3(light.intensity));
    G_fill_circle(SPREAD_VEC2(light_center), 5);
    //TODO: visualize the radius here
}