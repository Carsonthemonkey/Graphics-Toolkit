#include "raytrace.h"
#include <stdio.h>
#include <math.h>
#include "matrix.h"
#include "FPToolkit.h"
#include "colors.h"

int MAX_BOUNCES = 1;

void raytrace_scene(int width, int height, Camera cam, RaytracedParametricObject3D* objs, int num_objs){
    for(int y = 0; y < height; y++){
        double y_degrees = ((y / (double)height) * cam.half_fov_degrees * 2) - cam.half_fov_degrees;
        for(int x = 0; x < width; x++){
            double x_degrees = ((x / (double)width) * cam.half_fov_degrees * 2) - cam.half_fov_degrees;
            Vector3 camera_space_direction = {cos(x_degrees), sin(y_degrees), 1};
            vec3_normalize(&camera_space_direction);
            Vector3 world_space_direction = vec3_sub(mat4_mult_point(camera_space_direction, cam.inverse_view_matrix), cam.eye);
            Ray ray = {
                .origin=cam.eye,
                .direction=world_space_direction
            };

        //     double film_distance = get_film_distance(cam);
        //     Vector3 pixel_film_space = {
        //         (x / (double)width) - 0.5,
        //         (y / (double)height) - 0.5,
        //         film_distance
        //     };
        //     Vector3 pixel_direction = vec3_normalized(vec3_sub(mat4_mult_point(pixel_film_space, cam.inverse_view_matrix), cam.eye));
            G_rgb(SPREAD_COL3(world_space_direction));
            G_pixel(x, y);

        //     Ray ray = {.origin=cam.eye, .direction=pixel_direction};
            RayHitInfo hit = raytrace(ray, 1, objs, num_objs);
            if(!isnan(hit.location.x)){
                G_rgb(SPREAD_COL3(hit.color));
                G_pixel(x, y);
            }
        }
    }
}

RayHitInfo raytrace(Ray ray, int depth, RaytracedParametricObject3D* objs, int num_objs){
    double closest_t = INFINITY;
    RayHitInfo result = {
        .location={NAN, NAN, NAN},
        .normal={NAN, NAN, NAN},
        .color={BLACK}
    };

    for(int o = 0; o < num_objs; o++){
        RaytracedParametricObject3D object = objs[o];
        // Vector3 tsource = mat4_mult_point(ray.origin, object.inverse);
        // Vector3 tsource 
        // Vector3 ttip = mat4_mult_point(ray.direction, object.inverse);

        // x^2 + y^2 + z^2 - 1 = 0
        // (tsrc.x + tdir.x * t)^2 + (tsrc.y + tdir.y * t)^2 + (tsrc.z + tdir.x * z)^2 - 1 = 0
        // Now we need to foil the terms:
        //(tsrc * tsrc)
        Vector3 tip_dir = ray.direction; //vec3_sub(ttip, tsource);
        Vector3 tsource = ray.origin;
        Vector3 va = vec3_mult(tip_dir, tip_dir);
        // (tdir * tsrc) + (tsrc * tdir)
        Vector3 vb = vec3_scale(vec3_mult(tip_dir, tsource), 2);
        // (tdir * tdir)
        Vector3 vc = vec3_mult(tsource, tsource);
        //Add terms together
        double a = va.x + va.y + va.z;
        double b = vb.x + vb.y + vb.z;
        double c = vc.x + vc.y + vc.z - 1;

        //
        double under_sqrt = (b * b) - (4 * a * c);
        if(under_sqrt < 0) continue; // did not intersect with object
        double t_plus = (-b + sqrt(under_sqrt)) / (2 * a);
        double t_minus = (-b - sqrt(under_sqrt)) / (2 * a);

        double t;
        if(t_plus < 0 && t_minus < 0) continue; // object is behind camera
        if(t_plus > 0 && (t_plus < t_minus || t_minus < 0)) t = t_plus;
        else t = t_minus;

        if(t < closest_t && t > 0){
            closest_t = t;
            result.location = vec3_add(ray.origin, vec3_scale(ray.direction, t));
            //TODO: make this use materials instead
            result.color = object.color;
            //TODO: compute normal here
        }
    }
    return result;
}
