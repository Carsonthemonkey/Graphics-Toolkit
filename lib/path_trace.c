#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>
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
const int NUM_SHADOW_RAYS = 4;
const int NUM_CAMERA_RAYS = 256;
const int MAX_DIFFUSE_BOUNCES = 4;

bool intersect_triangle(double* t_out, Vector2* barycentric_out, double closest_t, Ray ray, Triangle triangle){
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
        if(barycentric_out != NULL){
            barycentric_out->x = u;
            barycentric_out->y = v;
        }
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

bool hits_anything(PathTracedScene scene, Ray ray, double distance_limit){
    for(int m = 0; m < scene.num_meshes; m++){
        Mesh mesh = scene.meshes[m];
        double t;
        if(!intersects_bounding_box(mesh, ray)) continue;
        for(int tr = 0; tr < mesh.num_tris; tr++){
            Triangle tri = mesh.tris[tr];
            if(intersect_triangle(NULL,NULL, distance_limit, ray, tri)) return true;
        }
    }
    return false;
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
            Vector2 surface_coords;
            if(intersect_triangle(&t,(mesh.shade_smooth ? &surface_coords : NULL), closest_t, ray, tri)){
                if(out == NULL) return true;

                did_hit = true;
                closest_t = t;
                out->intersected_mesh = mesh;
                out->distance = t;
                // Compute smooth normal
                if(mesh.shade_smooth){
                    Vector3 smooth_normal = vec3_add(
                        vec3_scale(tri.b->normal, surface_coords.x),
                        vec3_scale(tri.c->normal, surface_coords.y)
                    );
                    smooth_normal = vec3_add(smooth_normal, vec3_scale(tri.a->normal, 1 - surface_coords.x - surface_coords.y));
                    out->normal = smooth_normal;
                }
                else{
                    out->normal = tri.normal;
                }
            }
        }
    }
    return did_hit;
}

Vector3 ray_hit_location(Ray ray, double distance){
    return vec3_add(ray.origin, vec3_scale(ray.direction, distance));
}

//TODO: add different kinds of light support here 
Color3 direct_lighting(PathTracedScene scene, Vector3 position, Vector3 surface_normal){
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

            if(!hits_anything(scene, shadow, light_distance)){
                // Lambert's Cosine Law 
                double incident_dot = fmax(vec3_dot_prod(surface_normal, shadow.direction), 0);
                shadow_ray_hit_percentage += 1.0 / NUM_SHADOW_RAYS * incident_dot;
            } 
        }
        //Scale according to inverse square
        Color3 contribution = vec3_scale(vec3_scale(light.intensity, shadow_ray_hit_percentage), 1.0 / (light_distance * light_distance));
        direct_light = vec3_add(contribution, direct_light);
    }
    return direct_light;
}


Color3 path_trace(PathTracedScene scene, Ray ray, int depth){
    Color3 pixel_color = {0, 0, 0};
    Color3 throughput = {1, 1, 1};
    for(int r = 0; r < depth; r++){
        RayHitInfo hit;
        bool did_hit = raycast(&hit, scene, ray);
        if(!did_hit){
            // Environment lighting goes here
            pixel_color = vec3_add(pixel_color, (Color3){BLACK});
            break;
        }
        Mesh mesh = hit.intersected_mesh;
        Vector3 hit_location =  vec3_add(ray.origin, vec3_scale(ray.direction, hit.distance));

        /* Emissive Materials */
        pixel_color = vec3_add(pixel_color, vec3_mult(throughput, vec3_scale(mesh.material.emissive, mesh.material.emission_strength)));

        /* Direct Lighting */
        pixel_color = vec3_add(pixel_color, vec3_mult(vec3_mult(direct_lighting(scene, hit_location, hit.normal), mesh.material.base_color), throughput));
        
        /* Indirect Lighting */
        ray.origin = hit_location;
        if(rand_double() < mesh.material.specular){
            /* Specular Reflection */
            throughput = vec3_mult(throughput, mesh.material.specular_color);
            Vector3 diffuse_direction;
            if(mesh.material.roughness > 0) diffuse_direction = vec3_normalized(vec3_add(random_point_in_sphere(1), hit.normal));
            //TODO: Apparently interpolating the normal instead of the ray direction is better
            Vector3 reflection = vec3_reflection(ray.direction, hit.normal);
            Vector3 specular_direction = vec3_normalized(vec3_lerp(reflection, diffuse_direction, mesh.material.roughness));
            ray.direction = specular_direction;
        }
        else{
            /* Diffuse */
            throughput = vec3_mult(throughput, mesh.material.base_color);
            // This is the lambertian diffuse model / BRDF
            Vector3 diffuse_direction = vec3_normalized(vec3_add(random_point_in_sphere(1), hit.normal));
            ray.direction = diffuse_direction;
        }
        // Russian Roulette ray termination
        double ray_strength = fmax(fmax(throughput.r, throughput.g), throughput.b);
        if(rand_double() > ray_strength) break;
        throughput = vec3_scale(throughput, 1.0 / ray_strength);
    }

    return pixel_color;
}

void add_sample(Color3* buffer, Color3 pixel, int width, int x, int y, int sample_num){
    Color3 prev = get_pixel(buffer, width, x, y);
    double contribution = 1.0 / sample_num;
    Color3 new_color = vec3_add(vec3_scale(prev, 1.0 - contribution), vec3_scale(pixel, contribution));
    set_pixel(buffer, new_color, width, x, y);
}

void path_trace_scene(PathTracedScene scene, int y_start, int y_end){
    double dwidth = (double)scene.width;
    double dheight = (double)scene.height;
    //TODO: obligatory "make this work for non square aspect ratios"
    double film_extent = tan(to_radians(scene.main_camera->half_fov_degrees));
    for(int sample = 0; sample < NUM_CAMERA_RAYS; sample++){
        for(int y = y_start; y < y_end; y++){
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
                Color3 pixel_color = {0, 0, 0};
                Vector3 focal_point = vec3_add(scene.main_camera->eye, vec3_scale(world_space_dir, scene.main_camera->focal_length));

                /* Depth of Field and Anti-Aliasing */
                Vector2 jitter = random_point_in_circle(scene.main_camera->depth_of_field);
                Ray jittered_ray = {
                    .origin=vec3_add(ray.origin, 
                        vec3_sub(mat4_mult_point((Vector3){SPREAD_VEC2(jitter), 0}, scene.main_camera->inverse_view_matrix),
                        scene.main_camera->eye)
                    ),
                    .direction=vec3_normalized(vec3_sub(focal_point, jittered_ray.origin))
                };
                pixel_color = vec3_add(pixel_color, path_trace(scene, jittered_ray, MAX_DIFFUSE_BOUNCES));
                add_sample(scene.screen_buffer, pixel_color, scene.width, x, y, sample + 1);
            }
        }
    }
}

Color3* create_screen_buffer(int width, int height){
    Color3* buffer = (Color3*)malloc(sizeof(Color3) * width * height);
    if(buffer == NULL){
        fprintf(stderr, "Failed to allocate memory for screen buffer\n");
        exit(1);
    }
    return buffer;
}

Color3 get_pixel(Color3* screen_buffer, int width, int x, int y){
    return screen_buffer[(width * y) + x];
}

void set_pixel(Color3* screen_buffer, Color3 pixel, int width, int x, int y){
    screen_buffer[(width * y) + x] = pixel;
}

//TODO: Apply these color transforms elsewhere? so we can actually save the image easier. Maybe at the end of `path_trace_scene_multithreaded` or `path_trace_scene`
void draw_screen_buffer(PathTracedScene scene){
    for(int y = 0; y < scene.height; y++){
        for (int x = 0; x < scene.width; x++){
            Color3 color = vec3_scale(get_pixel(scene.screen_buffer, scene.width, x, y), scene.exposure);
            if(scene.tonemap != NULL) color = scene.tonemap(color);
            if(scene.color_transform != NULL) color = scene.color_transform(color);
            G_rgb(SPREAD_COL3(color));
            G_pixel(x, y);
        }
    }
}

void clear_screen_buffer(Color3* screen_buffer, Color3 color, int width, int height){
    for (int y = 0; y < height; y++){
        for (int x = 0; x < width; x++){
            set_pixel(screen_buffer, color, width, x, y);
        }
    }
}

struct PathTracingThreadInfo {
    PathTracedScene scene;
    int y_start;
    int y_end;
};

void* run_render_thread(void* path_tracing_thread_info){
    struct PathTracingThreadInfo info = *(struct PathTracingThreadInfo*)path_tracing_thread_info;
    path_trace_scene(info.scene, info.y_start, info.y_end);
    return 0;
}

volatile bool draw_buffer = true;

void* live_draw_buffer(void* path_tracing_thread_info){
    struct PathTracingThreadInfo info = *(struct PathTracingThreadInfo*)path_tracing_thread_info;
    while(draw_buffer){
        draw_screen_buffer(info.scene);
        G_display_image();
        //TODO: allow for a delay here
    }
    return 0;
}

const bool LIVE_DRAW = true;

void path_trace_scene_multithreaded(PathTracedScene scene){
    int num_threads = scene.height;
    pthread_t threads[num_threads];
    struct PathTracingThreadInfo thread_args[num_threads];

    for(int t = 0; t < num_threads; t++){
        // set up args
        thread_args[t].scene = scene;
        // Fix rounding errors here
        thread_args[t].y_start = (int)(t * ((double)scene.height / num_threads));
        thread_args[t].y_end = (int)((t + 1) * ((double)scene.height / num_threads));

        pthread_create(&threads[t], NULL, run_render_thread, (void*)&thread_args[t]);
    }
    pthread_t draw_thread;
    if(LIVE_DRAW){
        pthread_create(&draw_thread, NULL, live_draw_buffer, (void*)&thread_args[0]); // pass it any info struct for now
    }
    // Join threads
    for(int t = 0; t < num_threads; t++){
        pthread_join(threads[t], NULL);
    }
    if(LIVE_DRAW){
        draw_buffer = false;
        pthread_join(draw_thread, NULL);
        draw_buffer = true;
    }
    else{
        draw_screen_buffer(scene);
    }
}

void debug_draw_light(PointLight light, Camera cam, double width, double height){
    Vector2 light_center;
    if(!point_to_window(&light_center, light.position, cam, width, height)) return;
    G_rgb(SPREAD_VEC3(light.intensity));
    G_fill_circle(SPREAD_VEC2(light_center), 5);
    //TODO: visualize the radius here
}