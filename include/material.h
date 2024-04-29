#ifndef MATERIAL_H
#define MATERIAL_H
#include "colors.h"
#include "texture.h"
#include "mesh.h"
#include "path_trace.h"


/**
 * @brief Get the albedo color of a material at a given hit point
 * 
 * @param material The material to get the color from
 * @param hit_info The hit info of the ray
 * @return Color3 The albedo color of the material at the intersection point
 */
Color3 get_albedo_color(PathTracedMaterial material, RayHitInfo hit);

#endif