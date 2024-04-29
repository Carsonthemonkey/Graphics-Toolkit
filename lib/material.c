#include "material.h"
#include <math.h>
#include "vector.h"
#include "random.h"

Color3 eval_lambertian(Vector3 view, Vector3 normal, Vector3 light, PathTracedMaterial material) {
    double n_dot_l = fmax(vec3_dot_prod(normal, light), 0.0);
    return vec3_scale(material.base_color, n_dot_l);
}

Vector3 sample_lambertian(Vector3 surface_normal, PathTracedMaterial material_unused){
    return vec3_add(surface_normal, random_point_in_sphere(1));
}

double pdf_lambertian(double n_dot_l) {
    return n_dot_l / M_PI;
}

Vector3 eval_indirect_lambertian(double* sample_weight, Vector3 surface_normal, PathTracedMaterial material){
    Vector3 sample_dir = sample_lambertian(surface_normal, material);
    *sample_weight = 1.0;
    return sample_dir;
}
