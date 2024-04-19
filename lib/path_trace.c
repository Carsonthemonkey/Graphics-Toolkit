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

const double EPSILON = 0.000001;

bool intersect_triangle(Vector3* location_out, Ray ray, Triangle triangle){
    //TODO: precompute triangle normal
    //I don't really understand this intuitively. It would be a good idea to go back to this to get a better grasp on it
    Vector3 edge_1 = vec3_sub(triangle.b->position, triangle.a->position);
    Vector3 edge_2 = vec3_sub(triangle.c->position, triangle.a->position);

    Vector3 normal = vec3_cross_prod(edge_1, edge_2); //This may need to be normalized
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

    if(t > EPSILON) {
        Vector3 location = vec3_add(ray.origin, vec3_scale(ray.direction, t));
        *location_out = location;
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

            Vector3 dummy_location;
            // loop over all meshes
            G_rgb(SPREAD_COL3(ray.direction));
            for(int m = 0; m < scene.num_meshes; m++){
                // check against bounding box
                if(!intersects_bounding_box(scene.meshes[m], ray))continue;
                else G_rgb(RED);

                // loop over mesh tris
                for(int t = 0; t < scene.meshes[m].num_tris; t++){
                    //TODO: check which is closer
                    if(intersect_triangle(&dummy_location, ray, scene.meshes[m].tris[t])) G_rgb(GREEN); 
                }
            }

            // Triangle test_triangle = scene.meshes[0].tris[0];
            G_pixel(x, y);
        }
    }
}