#include "raytrace.h"
#include <stdio.h>
#include <math.h>
#include "matrix.h"
#include "FPToolkit.h"
#include "colors.h"
#include "trig.h"
#include "lightmodel.h"

int MAX_BOUNCES = 1;

void raytrace_scene(int width, int height, Camera cam, RaytracedParametricObject3D* objs, int num_objs){
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
            RayHitInfo hit = raytrace(ray, MAX_BOUNCES, objs, num_objs);
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
            Vector3 obj_space_location = vec3_add(tsource, vec3_scale(tdir, t));
            result.location = vec3_add(ray.origin, vec3_scale(ray.direction, t));


            Vector3 obj_normal = vec3_scale(obj_space_location, 2);
            result.normal.x = obj_normal.x * object.inverse[0][0] + obj_normal.y * object.inverse[1][0] + obj_normal.z * object.inverse[2][0];
            result.normal.y = obj_normal.x * object.inverse[0][1] + obj_normal.y * object.inverse[1][1] + obj_normal.z * object.inverse[2][1];
            result.normal.z = obj_normal.x * object.inverse[0][2] + obj_normal.y * object.inverse[1][2] + obj_normal.z * object.inverse[2][2];


            
            //TODO: make this use materials instead
            // result.color = object.color;
            result.color = result.normal;
            //TODO: compute normal here
        }
    }
    return result;
}
