#include "path_trace.h"
#include "FPToolkit.h"
#include "vector.h"

const double EPSILON = 0.00001;

bool intersect_triangle(Vector3* location_out, Ray ray, Triangle triangle){
    //TODO: precompute triangle normal
    //I don't really understand this intuitively. It would be a good idea to go back to this to get a better grasp on it
    Vector3 edge_1 = vec3_sub(triangle.b->position, triangle.a->position);
    Vector3 edge_2 = vec3_sub(triangle.c->position, triangle.a->position);
    Vector3 direction_normal_2 = vec3_cross_prod(edge_2, ray.direction); //This may need to be normalized
    double determinant = vec3_dot_prod(ray.direction, direction_normal_2);
    if(determinant > -EPSILON && determinant < EPSILON) return false; //Ray is paralell to triangle
    
    double inverse_determinant = 1.0 / determinant;
    Vector3 vertex_to_origin = vec3_sub(triangle.a->position, ray.origin);

    double u = vec3_dot_prod(vertex_to_origin, direction_normal_2) * inverse_determinant;
    if(u < 0 || u > 1) return false;

    Vector3 edge_1_cross_prod = vec3_cross_prod(vertex_to_origin, edge_1);
    double v = vec3_dot_prod(ray.direction, edge_1_cross_prod);
    if(v < 0 || v > 1) return false;

    double t = vec3_dot_prod(edge_2, edge_1_cross_prod) * inverse_determinant;

    if(t < EPSILON) {
        Vector3 location = vec3_add(ray.origin, vec3_scale(ray.direction, t));
        *location_out = location;
        return true;
    }
    return false;
}   