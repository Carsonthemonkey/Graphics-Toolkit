#include "vector.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

const Vector3 VEC3_ZERO    = {0, 0, 0};
const Vector3 VEC3_FORWARD = {0, 0, 1};
const Vector3 VEC3_BACK    = {0, 0, -1};
const Vector3 VEC3_UP      = {0, 1, 0};
const Vector3 VEC3_DOWN    = {0, -1, 0};
const Vector3 VEC3_LEFT    = {-1, 0, 0};
const Vector3 VEC3_RIGHT   = {1, 0, 0};

const Vector2 VEC2_ZERO = {0, 0};
const Vector2 VEC2_UP = {0, 1};
const Vector2 VEC2_DOWN = {0, -1};
const Vector2 VEC2_LEFT = {-1, 0};
const Vector2 VEC2_RIGHT = {1, 0};

/* 3D Vector Functions */

void vec3_print(Vector3 v){
    printf("<%lf, %lf, %lf>\n", v.x, v.y, v.z);
}

bool vec3_is_equal(Vector3 a, Vector3 b){
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

double vec3_magnitude(Vector3 v){
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

void vec3_normalize(Vector3 *v){
    double magnitude = vec3_magnitude(*v);
    v->x /= magnitude;
    v->y /= magnitude;
    v->z /= magnitude;
}

Vector3 vec3_normalized(Vector3 v){
    double magnitude = vec3_magnitude(v);
    Vector3 result;
    result.x = v.x / magnitude;
    result.y = v.y / magnitude;
    result.z = v.z / magnitude;
    return result;
}

Vector3 vec3_add(Vector3 a, Vector3 b){
    Vector3 result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

Vector3 vec3_negated(Vector3 v){
    return vec3_scale(v, -1);
}

Vector3 vec3_sub(Vector3 a, Vector3 b){
    Vector3 result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;
    return result;
}

Vector3 vec3_mult(Vector3 a, Vector3 b){
    Vector3 result;
    result.x = a.x * b.x;
    result.y = a.y * b.y;
    result.z = a.z * b.z;
    return result;
}

Vector3 vec3_div(Vector3 a, Vector3 b){
    Vector3 result;
    result.x = a.x / b.x;
    result.y = a.y / b.y;
    result.z = a.z / b.z;
    return result;
}

Vector3 vec3_scale(Vector3 v, double scale){
    Vector3 result;
    result.x = v.x * scale;
    result.y = v.y * scale;
    result.z = v.z * scale;
    return result;
}

double vec3_dot_prod(Vector3 a, Vector3 b){
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

Vector3 vec3_cross_prod(Vector3 a, Vector3 b){
    Vector3 result;
    result.x = (a.y * b.z) - (b.y * a.z);
    result.y = (a.z * b.x) - (b.z * a.x);
    result.z = (a.x * b.y) - (b.x * a.y);
    return result;
}

double vec3_distance(Vector3 a, Vector3 b){
    return vec3_magnitude(vec3_sub(b, a));
}

Vector3 vec3_lerp(Vector3 a, Vector3 b, double t){
    Vector3 result;
    result.x = a.x + (b.x - a.x) * t;
    result.y = a.y + (b.y - a.y) * t;
    result.z = a.z + (b.z - a.z) * t;
    return result;
}

Vector3 vec3_reflection(Vector3 incoming, Vector3 normal){
    Vector3 result;
    double dot_prod = vec3_dot_prod(incoming, normal);
    Vector3 scaled_normal = vec3_scale(normal, 2 * dot_prod);
    result = vec3_sub(incoming, scaled_normal);
    return result;
}

Vector3f vec3_to_vec3f(Vector3 v){
    return (Vector3f){
        .x=(float)v.x,
        .y=(float)v.y,
        .z=(float)v.z,
    };
}

Vector3 vec3f_to_vec3(Vector3f v){
    return (Vector3){
        .x=(double)v.x,
        .y=(double)v.y,
        .z=(double)v.z,
    };
}

Vector3f vec3f_add(Vector3f a, Vector3f b){
    Vector3f result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

Vector3f vec3f_scale(Vector3f v, double scale){
    Vector3f result;
    result.x = v.x * scale;
    result.y = v.y * scale;
    result.z = v.z * scale;
    return result;
}

double* vec3_to_array(Vector3* vec){
    //exists so that it's behaviour can be updated later
    return (double*)vec;
}

/* 2D Vector Functions */

void vec2_print(Vector2 v){
    printf("<%lf, %lf>\n", v.x, v.y);
}

double vec2_magnitude(Vector2 v){
    return sqrt(v.x * v.x + v.y * v.y);
}

void vec2_normalize(Vector2 *v){
    double magnitude = vec2_magnitude(*v);
    v->x /= magnitude;
    v->y /= magnitude;
}

Vector2 vec2_normalized(Vector2 v){
    double magnitude = vec2_magnitude(v);
    Vector2 result;
    result.x = v.x / magnitude;
    result.y = v.y / magnitude;
    return result;
}

Vector2 vec2_add(Vector2 a, Vector2 b){
    Vector2 result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    return result;
}

Vector2 vec2_sub(Vector2 a, Vector2 b){
    Vector2 result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    return result;
}

Vector2 vec2_mult(Vector2 a, Vector2 b){
    Vector2 result;
    result.x = a.x * b.x;
    result.y = a.y * b.y;
    return result;
}

Vector2 vec2_div(Vector2 a, Vector2 b){
    Vector2 result;
    result.x = a.x / b.x;
    result.y = a.y / b.y;
    return result;
}

Vector2 vec2_scale(Vector2 v, double scale){
    Vector2 result;
    result.x = v.x * scale;
    result.y = v.y * scale;
    return result;
}

double vec2_dot_prod(Vector2 a, Vector2 b){
    return (a.x * b.x) + (a.y * b.y);
}


