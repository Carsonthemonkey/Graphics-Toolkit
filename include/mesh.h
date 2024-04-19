#ifndef MESH_H
#define MESH_H
#include "vector.h"


typedef struct {
    Vector3 position;
    Vector3 normal;
} Vertex;

typedef struct {
    // a, b, and c are indices of vertices in the mesh
    int a, b, c;
} Triangle;


typedef struct {
    int num_tris;
    Triangle* tris;
    int num_vertices;
    Vertex* vertices;

    double transform[4][4];
    double inverse_transform[4][4];
} Mesh;

/**
 * @brief Loads triangles from an ascii .ply file.
 * Assumes that the mesh is triangulated and that it has precomputed vertex normals only.
 * 
 * @param filename The name of the file to be loaded
 * @return Triangle* The triangles loaded from the stl file
 */
Mesh load_mesh_from_ply(char* filename);

/**
 * @brief Frees all mesh vertices and faces from memory
 * 
 * @param mesh the mesh to be deleted
 */
void delete_mesh(Mesh mesh);

#endif