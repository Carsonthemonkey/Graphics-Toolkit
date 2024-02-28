#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "parametric.h"
#include "vector.h"
#include "matrix.h"
#include "camera.h"
#include "lightmodel.h"
#include "FPToolkit.h"
#include "texture.h"
#include "xwd_tools.h"

static const bool BACKFACE_CULLING = false; //TODO: Make this an option that can be passed in?

double NORMAL_DELTA = 0.001;


void draw_parametric_object_3d(ParametricObject3D object,
                            Camera cam,
                            PhongLight* lights,
                            int num_lights,
                            int width, int height,
                            double z_buffer[width][height],
                            enum ViewMode mode)
{
    for(double u = object.u_start; u < object.u_end; u += object.u_step){
        for(double v = object.v_start; v < object.v_end; v += object.v_step){

            Vector3 point = mat4_mult_point(object.f(u, v), object.transform);

            Vector3 camera_point = to_camera_space(point, cam);
            if(!is_visible_to_camera(cam, camera_point)) continue; //Cull point if not visible
            
            double normalized_z_dist = (camera_point.z - cam.near_clip_plane) / (cam.far_clip_plane - cam.near_clip_plane);
            Vector2 pixel_location = to_window_coordinates(to_camera_screen_space(camera_point, cam), width, height);
            

            if(normalized_z_dist >  z_buffer[(int)pixel_location.x][(int)pixel_location.y]) continue;

            z_buffer[(int)pixel_location.x][(int)pixel_location.y] = normalized_z_dist;

            Vector3 tangent_a;
            Vector3 tangent_b;
            Vector3 normal;
            if(BACKFACE_CULLING){ //TODO: Make this more optimized. View vector can be reused in phong lighting.
                tangent_a = vec3_normalized(vec3_sub(mat4_mult_point(object.f(u, v + NORMAL_DELTA), object.transform), point));
                tangent_b = vec3_normalized(vec3_sub(mat4_mult_point(object.f(u + NORMAL_DELTA, v), object.transform), point));
                normal = vec3_cross_prod(tangent_a, tangent_b);
                Vector3 view_vec = vec3_normalized(vec3_sub(cam.eye, point));
                if(vec3_dot_prod(normal, view_vec) < 0) {
                    continue; //cull if cant see
                } 

            }
            //Apply texture
            if(object.material.texture.data.xwd_texture_id >= 0 && (mode == UNLIT || mode == LIT)){
                double u_range = object.u_end - object.u_start;
                double v_range = object.v_end - object.v_start;
                Vector2 uv = {u / u_range * object.material.texture.width, v / v_range * object.material.texture.height};
                Color3 tex_col = get_texture_color(object.material.texture, uv);
                object.material.base_color = tex_col;
                object.material.diffuse = tex_col;
            }
            if(mode == UNLIT){
                G_rgb(SPREAD_COL3(object.material.base_color));
            } else if(mode == UV){
                double u_range = object.u_end - object.u_start;
                double v_range = object.v_end - object.v_start;
                G_rgb(u / u_range, v / v_range, 0);
            } 
            else if(mode == Z_BUFF){
                double brightness = z_buffer[(int)pixel_location.x][(int)pixel_location.y];
                G_rgb(brightness, brightness, brightness);
            } else{
                /* Do lighting calculations */
                if(!BACKFACE_CULLING){ //If backface culling is true, normal is already computed
                    tangent_a = vec3_normalized(vec3_sub(mat4_mult_point(object.f(u, v + NORMAL_DELTA), object.transform), point));
                    tangent_b = vec3_normalized(vec3_sub(mat4_mult_point(object.f(u + NORMAL_DELTA, v), object.transform), point));
                    normal = vec3_cross_prod(tangent_a, tangent_b);
                }

                if(mode == NORMAL){
                    G_rgb(SPREAD_COL3(normal));
                } else if (mode == LIT){
                    Color3 col = phong_lighting(point, normal, cam, object.material, lights, num_lights);
                    G_rgb(SPREAD_COL3(col));
                }
            }
            draw:
            G_pixel(SPREAD_VEC2(pixel_location));
        }
    }
}

void draw_parametric_objects_3d(ParametricObject3D* objects,
                                int num_objs,
                                Camera cam,
                                PhongLight* lights,
                                int num_lights,
                                int width,
                                int height,
                                double z_buffer[width][height],
                                enum ViewMode mode)
{
    for(int o = 0; o < num_objs; o++){
        draw_parametric_object_3d(objects[o], cam, lights, num_lights, width, height, z_buffer, mode);
    }
}


Vector3 param_sphere(double u, double v){
    Vector3 result;
    result.x = cos(u) * sin(v);
    result.y = sin(u) * sin(v);
    result.z = cos(v);
    return result;
}

Vector3 param_plane(double u, double v){
    Vector3 result = {u, 0, v};
    return result;
}
    
Vector3 param_cylinder(double u, double v){
    Vector3 result; 
    result.x = cos(u);
    result.y = sin(-u);
    result.z = v;
    return result;
}

double TORUS_MINOR_RADIUS = 0.3;
double TORUS_MAJOR_RADIUS = 1;

//TODO: we need some way to be able to specify the major and minor radius here. Hm
Vector3 param_torus(double u, double v){
    Vector3 result;
    result.x = (TORUS_MAJOR_RADIUS + TORUS_MINOR_RADIUS * cos(u)) * cos(v);
    result.y = (TORUS_MAJOR_RADIUS + TORUS_MINOR_RADIUS * cos(u)) * sin(v);
    result.z = TORUS_MINOR_RADIUS * sin(u);
    return result;
}