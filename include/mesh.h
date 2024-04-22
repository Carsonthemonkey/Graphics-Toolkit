#ifndef MESH_H
#define MESH_H
#include "vector.h"
#include "camera.h"
#include "colors.h"

//TODO: put this in path_trace.h but its causing include errors right now
typedef struct {
    Color3 base_color;
    Color3 emissive;
    double emission_strength;
    double specular;
    Color3 specular_color;
    double roughness;
} PathTracedMaterial;

typedef struct {
    Vector3 position;
    Vector3 normal;
} Vertex;

typedef struct {
    // a, b, and c are indices of vertices in the mesh
    Vertex* a;
    Vertex* b;
    Vertex* c;
    Vector3 normal;
} Triangle;


typedef struct {
    int num_tris;
    Triangle* tris;
    int num_vertices;
    Vertex* vertices;

    Vector3 bounding_box_max;
    Vector3 bounding_box_min;

    bool shade_smooth;
    PathTracedMaterial material;

    double transform[4][4];
    double inverse_transform[4][4];

} Mesh;

/**
 * @brief Loads triangles from an ascii .ply file into a given mesh struct.
 * Assumes that the mesh is triangulated and that it has precomputed vertex normals only.
 * 
 * @param mesh A pointer to the mesh to load the data into
 * @param filename The name of the file to be loaded
 */
void load_mesh_from_ply(Mesh* mesh, char* filename);

/**
 * @brief Frees all mesh vertices and faces from memory
 * 
 * @param mesh the mesh to be deleted
 */
void delete_mesh(Mesh mesh);

/**
 * @brief Translates a mesh's transform by the given translation.
 * 
 * @param mesh The mesh to be translated
 * @param translation The translation vector to use
 */
void translate_mesh(Mesh* mesh, Vector3 translation);

/**
 * @brief Scales a mesh's transform by the given scaling factors.
 * 
 * @param mesh The mesh to be scaled
 * @param scale The scale vector to use 
 */
void scale_mesh(Mesh* mesh, Vector3 scale);

/**
 * @brief Rotates a Mesh on the x axis
 * 
 * @param mesh the mesh to be rotated
 * @param degrees the degrees on the x axis to rotate the mesh by
 */

void rotate_mesh_x_degrees(Mesh* mesh, double degrees);
/**
 * @brief Rotates a Mesh on the y axis
 * 
 * @param mesh the mesh to be rotated
 * @param degrees the degrees on the y axis to rotate the mesh by
 */
void rotate_mesh_y_degrees(Mesh* mesh, double degrees);

/**
 * @brief Rotates a Mesh on the z axis
 * 
 * @param mesh the mesh to be rotated
 * @param degrees the degrees on the z axis to rotate the mesh by
 */
void rotate_mesh_z_degrees(Mesh* mesh, double degrees);

/**
 * @brief Computes the bounding box for a given mesh
 * 
 * @param mesh 
 */
void compute_mesh_bounds(Mesh* mesh);
 
 /**
  * @brief Computes and sets the normals for all faces in the mesh
  * 
  * @param mesh The mesh whose normals will be computed
  */
void compute_face_normals(Mesh* mesh);

/**
 * @brief Draws a mesh wireframe to the screen
 * 
 * @param mesh The mesh to draw
 * @param cam The camera from which to draw the mesh
 * @param width The width of the screen
 * @param height the height of the screen
 */
void debug_draw_mesh(Mesh mesh, Camera cam, int width, int height);
#endif