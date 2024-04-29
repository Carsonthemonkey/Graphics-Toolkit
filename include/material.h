#ifndef MATERIAL_H
#define MATERIAL_H

#include "colors.h"

typedef struct {
    Color3 base_color;
    Color3 emissive;
    double emission_strength;
    double specular;
    Color3 specular_color;
    double roughness;
} PathTracedMaterial;

#endif