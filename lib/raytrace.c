#include "raytrace.h"
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "matrix.h"
#include "FPToolkit.h"
#include "colors.h"
#include "trig.h"
#include "lightmodel.h"

int MAX_BOUNCES = 1;

void raytrace_scene(int width, int height, Camera cam, RaytracedParametricObject3D* objs, int num_objs, RaytracedLight* lights, int num_lights){
    //TODO: Make this work for non spheres
    double dwidth = (double)width;
    double dheight = (double)height;
    double film_extent = tan(to_radians(cam.half_fov_degrees));
    for(int y = 0; y < height; y++){
        for(int x = 0; x < width; x++){
            //TODO: make this work for different aspect ratios
            Vector3 pixel_camera_space = {
                ((x - (dwidth / 2)) / dwidth) * (film_extent * 2),
                ((y - (dheight / 2)) / dheight) * (film_extent * 2),
                1
            };

            Vector3 world_space_dir = vec3_sub(mat4_mult_point(pixel_camera_space, cam.inverse_view_matrix), cam.eye);
            Ray ray = {
                .origin=cam.eye,
                .direction=world_space_dir
            };
            G_rgb(SPREAD_COL3(vec3_normalized(world_space_dir)));
            G_pixel(x, y);
            RayHitInfo hit;
            if(raytrace(&hit, ray, MAX_BOUNCES, objs, num_objs, lights, num_lights)){
                G_rgb(SPREAD_COL3(hit.color));
                G_pixel(x, y);
            }
        }
    }
}

bool raytrace(RayHitInfo* out, Ray ray, int depth, RaytracedParametricObject3D* objs, int num_objs, RaytracedLight* lights, int num_lights){
    double closest_t = INFINITY;
    bool did_hit = false;
    // RayHitInfo result = {
    //     .location={NAN, NAN, NAN},
    //     .normal={NAN, NAN, NAN},
    //     .color={BLACK}
    // };
    Vector3 tip = vec3_add(ray.origin, ray.direction);
    for(int o = 0; o < num_objs; o++){
        RaytracedParametricObject3D object = objs[o];
        
        Vector3 tsource = mat4_mult_point(ray.origin, object.inverse);
        Vector3 tdir = vec3_sub(mat4_mult_point(tip, object.inverse), tsource);

        // Now we need to foil the terms:
        //(tsrc * tsrc)
        // Vector3 tip_dir = ray.direction; //vec3_sub(ttip, tsource);
        // Vector3 tsource = ray.origin;
        Vector3 va = vec3_mult(tdir, tdir);
        // (tdir * tsrc) + (tsrc * tdir)
        Vector3 vb = vec3_scale(vec3_mult(tdir, tsource), 2);
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
            if(out == NULL) return true;
            did_hit = true;

            Vector3 obj_space_location = vec3_add(tsource, vec3_scale(tdir, t));
            out->location = vec3_add(ray.origin, vec3_scale(ray.direction, t));
            Vector3 obj_normal = vec3_scale(obj_space_location, 2);
            out->normal.x = obj_normal.x * object.inverse[0][0] + obj_normal.y * object.inverse[1][0] + obj_normal.z * object.inverse[2][0];
            out->normal.y = obj_normal.x * object.inverse[0][1] + obj_normal.y * object.inverse[1][1] + obj_normal.z * object.inverse[2][1];
            out->normal.z = obj_normal.x * object.inverse[0][2] + obj_normal.y * object.inverse[1][2] + obj_normal.z * object.inverse[2][2];

            //Check if object is lit
            out->color = object.material.base_color;
            for(int l = 0; l < num_lights; l++){
                RaytracedLight light = lights[l];
                Vector3 light_dir = vec3_normalized(vec3_sub(light.position, out->location));
                Ray shadow_ray = {
                    .origin=vec3_add(out->location, vec3_scale(out->normal, 0.01)),
                    .direction=light_dir
                };
                //TODO: make this more generalized of a function
                if(raytrace(NULL, shadow_ray, 1, objs, num_objs, lights, num_lights)){
                    out->color = vec3_scale(out->color, 0.2);
                    break;
                }
            }
            
            //TODO: make this use materials instead
            
            // result.color = phong_lighting(result.location, result.normal)
            // result.color = result.normal;
            //TODO: compute normal here
        }
    }
    return did_hit;
}
