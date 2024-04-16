#include <math.h>
#include <stdio.h>
#include "trig.h"
#include "lightmodel.h"
#include "camera.h"
#include "vector.h"
#include "FPToolkit.h"

static const int LIGHT_GIZMO_RADIUS = 7;

const double AMBIENT = 0.2;

Color3 phong_lighting(Vector3 position, Vector3 normal, Camera cam, PhongMaterial material, PhongLight* lights, int num_lights){
    Color3 result = vec3_scale(material.base_color, AMBIENT);
    for(int l = 0; l < num_lights; l++){
        PhongLight light = lights[l];

        /* Diffuse */
        Vector3 light_dir = vec3_normalized(vec3_sub(light.position, position));
        double dot_prod = vec3_dot_prod(light_dir, normal);
        if(dot_prod < 0) continue;
        Color3 diffuse = vec3_mult(vec3_scale(light.diffuse, dot_prod), material.diffuse);
        result = vec3_add(result, diffuse);

        /* Specular */
        Vector3 reflection = vec3_normalized(vec3_sub(vec3_scale(normal, 2 * dot_prod), light_dir));
        Vector3 view_vec = vec3_normalized(vec3_sub(cam.eye, position));
        double spec = pow(vec3_dot_prod(reflection, view_vec), material.shininess);
        if(spec < 0) continue;
        Color3 specular = vec3_mult(vec3_scale(light.specular, spec), material.specular);
        result = vec3_add(result, specular);
    }
    return result;
}

Color3 phong_lighting_eye(Vector3 position, Vector3 normal, Vector3 eye, PhongMaterial material, PhongLight* lights, int num_lights){
    Color3 result = vec3_scale(material.base_color, AMBIENT);
    for(int l = 0; l < num_lights; l++){
        PhongLight light = lights[l];

        /* Diffuse */
        Vector3 light_dir = vec3_normalized(vec3_sub(light.position, position));
        double dot_prod = vec3_dot_prod(light_dir, normal);
        if(dot_prod < 0) continue;
        Color3 diffuse = vec3_mult(vec3_scale(light.diffuse, dot_prod), material.diffuse);
        result = vec3_add(result, diffuse);

        /* Specular */
        Vector3 reflection = vec3_normalized(vec3_sub(vec3_scale(normal, 2 * dot_prod), light_dir));
        Vector3 view_vec = vec3_normalized(vec3_sub(eye, position));
        double dp = fabs(vec3_dot_prod(reflection, view_vec));
        double spec = pow(dp, material.shininess);
        if(spec < 0) continue;
        Color3 specular = vec3_mult(vec3_scale(light.specular, spec), material.specular);
        result = vec3_add(result, specular);

        // if (isnan(result.x)) /*vec3_print(vec3_dot_prod(reflection, view_vec)); */ printf("%lf\n",vec3_dot_prod(reflection, view_vec)); 
        
    }
    // if(vec3_magnitude(result) > sqrt(3)) vec3_print(result);
    return result;
}

//TODO: Make this safe. 
//* because G_pixel is not safe (according to FPToolkit comments), this function can cause undefined behaviour 
//* if the lines that are drawn happen to go outside the window. This ca happen if the light is near the edge of the screen.
void draw_light(PhongLight light, Camera cam, int width, int height){
    Vector2 pixel_coords;
    bool visible = point_to_window(&pixel_coords, light.position, cam, width, height);
    if(!visible) return;
    G_rgb(SPREAD_COL3(light.diffuse));
    G_fill_circle(SPREAD_VEC2(pixel_coords), LIGHT_GIZMO_RADIUS);
    
    for(int a = 0; a < 360; a += 45){
        double rad_angle = to_radians(a);
        Vector2 angle_vec = {cos(rad_angle), sin(rad_angle)};
        Vector2 start = vec2_add(vec2_scale(angle_vec, LIGHT_GIZMO_RADIUS + 3), pixel_coords);
        Vector2 end = vec2_add(vec2_scale(angle_vec, LIGHT_GIZMO_RADIUS * 2), pixel_coords);
        G_line(start.x, start.y, end.x, end.y);
    }
}

void draw_lights(PhongLight* lights, int num_lights, Camera cam, int width, int height){
    for(int l = 0; l < num_lights; l++){
        draw_light(lights[l], cam, width, height);
    }
}
