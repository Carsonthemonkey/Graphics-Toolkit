#include "material.h"
#include "texture.h"
#include "mesh.h"

//TODO: I want to just pass in a RayHitInfo but I am getting circular dependencies from including path_trace.h
Color3 get_albedo_color(PathTracedMaterial material, RayHitInfo hit){
    if(!texture_is_null(material.albedo_texture)){
        Triangle hit_triangle = hit.intersected_triangle;
        Vector2 texture_coords;
        texture_coords.u = hit_triangle.b->uv.u * hit.surface_coords.x +
                           hit_triangle.c->uv.u * hit.surface_coords.y +
                           hit_triangle.a->uv.u * (1.0 - hit.surface_coords.x - hit.surface_coords.y);
        texture_coords.v = hit_triangle.b->uv.v * hit.surface_coords.x +
                           hit_triangle.c->uv.v * hit.surface_coords.y +
                           hit_triangle.a->uv.v * (1.0 - hit.surface_coords.x - hit.surface_coords.y);
        texture_coords = vec2_mult(texture_coords, (Vector2){material.albedo_texture.width, material.albedo_texture.height});
        return get_texture_color(material.albedo_texture, texture_coords);
    } else {    
        return material.base_color;
    }
}
