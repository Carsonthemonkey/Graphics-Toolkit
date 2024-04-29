#ifndef MATERIAL_H
#define MATERIAL_H

#include "colors.h"
#include "vector.h"

typedef struct {
    Color3 base_color;
    Color3 emissive;
    double emission_strength;
    double specular;
    Color3 specular_color;
    double roughness;
} PathTracedMaterial;

/* BRDFs */

Color3 eval_lambertian(Vector3 view, Vector3 normal, Vector3 light, PathTracedMaterial material);

Color3 sample_lambertian(Vector3 surface_normal, PathTracedMaterial material_unused);

// not sure if this is generalizable to all BRDFs
double pdf_lambertian(double n_dot_l);

Color3 eval_indirect_lambertian(double* sample_weight, Vector3 surface_normal, PathTracedMaterial material);

#endif