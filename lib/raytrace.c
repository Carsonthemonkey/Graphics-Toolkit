#include "raytrace.h"
#include <stdio.h>
#include <math.h>
#include "matrix.h"
#include "FPToolkit.h"
#include "colors.h"

int MAX_BOUNCES = 1;

void raytrace_scene(int width, int height, Camera cam, RaytracedParametricObject3D* objs, int num_objs){
    double step_x = 1 / (double)width;
    double step_y = 1 / (double)height;
    Vector3 tip = {-0.5, -0.5, get_film_distance(cam)};
    printf("%lf\n", get_film_distance(cam));
    for(tip.y = -0.5; tip.y < 0.5; tip.y += step_y){
        for(tip.x = -0.5; tip.x < 0.5; tip.x += step_x){
            // vec3_print(tip);
            // Vector3 direction = vec3_normalized(tip);
            Vector3 ttip = mat4_mult_point(tip, cam.view_matrix); // transform to camera space
            // G_rgb(ttip.x, ttip.y, 0);
            // G_pixel((tip.x + 0.5) * width, (tip.y + 0.5) * height);
            RayHitInfo hit = ray(cam.eye, ttip, 1, objs, num_objs);
            G_rgb(SPREAD_COL3(hit.color));
            G_pixel((tip.x + 0.5) * width, (tip.y + 0.5) * height);
        }
    }
}

RayHitInfo ray(Vector3 source, Vector3 tip, int depth, RaytracedParametricObject3D* objs, int num_objs){
    double closest_t = INFINITY;
    RayHitInfo result = {
        .location={NAN, NAN, NAN},
        .normal={NAN, NAN, NAN},
        .color={BLACK}
    };

    for(int o = 0; o < num_objs; o++){
        RaytracedParametricObject3D object = objs[o];
        Vector3 tsource = mat4_mult_point(source, object.inverse);
        Vector3 ttip = mat4_mult_point(tip, object.inverse);
        // x^2 + y^2 + z^2 - 1 = 0
        // (tsrc.x + tdir.x * t)^2 + (tsrc.y + tdir.y * t)^2 + (tsrc.z + tdir.x * z)^2 - 1 = 0
        // Now we need to foil the terms:
        //(tsrc * tsrc)
        Vector3 tip_dir = vec3_sub(ttip, tsource);
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
            result.location = vec3_add(source, vec3_scale(tip, t));
            //TODO: make this use materials instead
            result.color = object.color;
            //TODO: compute normal here
        }
    }
    return result;
}
