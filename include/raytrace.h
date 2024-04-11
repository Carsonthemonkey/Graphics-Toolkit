#ifndef RAYTRACE_H
#define RAYTRACE_H
#include "vector.h"
#include "camera.h"
#include "colors.h"
#include "lightmodel.h"

enum RaytracedObjectType {
    SPHERE
};

typedef struct {
    Vector3 origin;
    Vector3 direction;
} Ray;

typedef struct {
    Color3 base_color;
} RaytracedMaterial;

typedef struct {
    Vector3 position;
} RaytracedLight;

typedef struct {
    enum RaytracedObjectType object_type;
    Vector3 (*f)(double, double);
    double transform[4][4];
    double inverse[4][4];
    RaytracedMaterial material;
} RaytracedParametricObject3D;

typedef struct {
    Vector3 location;
    Vector3 normal;
    Color3 color;
} RayHitInfo;



void raytrace_scene(int width, int height, Camera cam, RaytracedParametricObject3D* objs, int num_objs, RaytracedLight* lights, int num_lights);

bool raytrace(RayHitInfo* out, Ray ray, int depth, RaytracedParametricObject3D* objs, int num_objs, RaytracedLight* lights, int num_lights);

#endif