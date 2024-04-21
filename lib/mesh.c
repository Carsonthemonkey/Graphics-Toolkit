#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include "mesh.h"
#include "FPToolkit.h"
#include "colors.h"
#include "M3d_matrix_tools.h"
#include "path_trace.h"
#include "trig.h"

const int BUFFER_SIZE = 256;

void trim_trailing_whitespace(char *str) {
    int i;

    // Null check
    if (str == NULL) {
        return;
    }

    i = strlen(str) - 1;
    while (i >= 0 && isspace((unsigned char)str[i])) {
        str[i] = '\0';
        i--;
    }
}

//TODO: It would be nice if this was more versatile. Right now it requires that the .ply file has the vertex positions followed by the vertex normals. It would be ideal if it could work without vertex normals or be in a different order or whatnot
void load_mesh_from_ply(Mesh* mesh, char* filename){
    // Read until element

    // Record element number and type
    // check if there are normals or assert there are
    // repeat until end_header
    // malloc space for vertices and tris
    // Read in points
    mesh->num_vertices = -1;
    mesh->num_tris = -1;
    char line[BUFFER_SIZE];
    char *token;

    FILE* f = fopen(filename, "r");
    if(f == NULL){
        fprintf(stderr, "No such file '%s'\n", filename);
        exit(1);
    }
    fgets(line, BUFFER_SIZE, f);
    trim_trailing_whitespace(line);
    assert(!strcmp(line, "ply"));
    while(fgets(line, BUFFER_SIZE, f) && (mesh->num_vertices ==  -1 || mesh->num_tris == -1)){
        token = strtok(line, " ");
        // Find the number of faces and vertices
        
        if(strcmp(token, "element")) continue;
        else{
            char* element_type = strtok(NULL, " ");
            int element_num = atoi(strtok(NULL, " "));
            if(!strcmp(element_type, "vertex")){
                mesh->num_vertices = element_num;
            }
            else if(!strcmp(element_type, "face")){
                mesh->num_tris = element_num;
            }
            else {
                fprintf(stderr, "Unknown element type '%s'\n", element_type);
                exit(1);
            }
        }
    }

    // Face num and vertex num have been recorded
    mesh->num_tris = mesh->num_tris;
    mesh->num_vertices = mesh->num_vertices; 
    mesh->vertices = (Vertex*)malloc(sizeof(Vertex) * mesh->num_vertices);
    if(mesh->vertices == NULL) goto MEM_ERROR;
    mesh->tris = (Triangle*)malloc(sizeof(Triangle) * mesh->num_tris);
    if(mesh->tris == NULL) goto MEM_ERROR;

    while(fgets(line, BUFFER_SIZE, f)){
        token = strtok(line, " ");
        trim_trailing_whitespace(token);
        if(!strcmp(token, "end_header")) break;
    }
    // printf("End header\n");
    // Now we read values
    int i = 0;
    while(i < mesh->num_vertices && fgets(line, BUFFER_SIZE, f)){
        // Read positions
        token = strtok(line, "\n\r\t ");
        if(!strcmp(token, "comment")) continue;
        mesh->vertices[i].position.x = strtod(token, NULL);
        token = strtok(NULL, "\n\r\t ");
        mesh->vertices[i].position.y = strtod(token, NULL);
        token = strtok(NULL, "\n\r\t ");
        mesh->vertices[i].position.z = strtod(token, NULL);
        
        //Read normals
        token = strtok(NULL, "\n\r\t ");
        mesh->vertices[i].normal.x = strtod(token, NULL);
        token = strtok(NULL, "\n\r\t ");
        mesh->vertices[i].normal.y = strtod(token, NULL);
        token = strtok(NULL, "\n\r\t ");
        mesh->vertices[i].normal.z = strtod(token, NULL);
        i++;
    }
    i = 0;
    while(i < mesh->num_tris && fgets(line, BUFFER_SIZE, f)){

        token = strtok(line, "\n\r\t ");
        if(!strcmp(token, "comment")) continue;
        if(atoi(token) != 3){
            fprintf(stderr, "faces must be triangles\n");
            exit(1);
        }
        token = strtok(NULL, "\n\r\t ");
        mesh->tris[i].a = mesh->vertices + atoi(token);
        token = strtok(NULL, "\n\r\t ");
        mesh->tris[i].b = mesh->vertices + atoi(token);
        token = strtok(NULL, "\n\r\t ");
        mesh->tris[i].c = mesh->vertices + atoi(token);
        i++;
    }


    fclose(f);
    return;
    MEM_ERROR:
    fprintf(stderr, "Failed to allocate sufficient memory for mesh\n");
    exit(1);
};

void delete_mesh(Mesh mesh){
    free(mesh.vertices);
    mesh.vertices = NULL;
    free(mesh.tris);
    mesh.tris = NULL;
}

void translate_mesh(Mesh* mesh, Vector3 translation){
    double transform[4][4];
    double inverse[4][4];
    M3d_make_translation(transform, SPREAD_VEC3(translation));
    M3d_make_translation(inverse, SPREAD_VEC3(vec3_scale(translation, -1)));

    M3d_mat_mult(mesh->transform, transform, mesh->transform);
    M3d_mat_mult(mesh->inverse_transform, inverse, mesh->inverse_transform);
}

void scale_mesh(Mesh* mesh, Vector3 scale){
    double transform[4][4];
    double inverse[4][4];
    M3d_make_scaling(transform, SPREAD_VEC3(scale));
    M3d_make_scaling(transform, SPREAD_VEC3(vec3_scale(scale, -1)));

    M3d_mat_mult(mesh->transform, transform, mesh->transform);
    M3d_mat_mult(mesh->transform, transform, mesh->transform);
}

void rotate_mesh_x_degrees(Mesh* mesh, double degrees){
    double transform[4][4];
    double inverse[4][4];
    double rads = to_radians(degrees);

    M3d_make_x_rotation_cs(transform, cos(degrees), sin(degrees));
    M3d_make_x_rotation_cs(transform, cos(degrees), -sin(degrees));

    M3d_mat_mult(mesh->transform, transform, mesh->transform);
    M3d_mat_mult(mesh->inverse_transform, inverse, mesh->inverse_transform);
}

void rotate_mesh_y_degrees(Mesh* mesh, double degrees){
    double transform[4][4];
    double inverse[4][4];
    double rads = to_radians(degrees);

    M3d_make_y_rotation_cs(transform, cos(degrees), sin(degrees));
    M3d_make_y_rotation_cs(transform, cos(degrees), -sin(degrees));

    M3d_mat_mult(mesh->transform, transform, mesh->transform);
    M3d_mat_mult(mesh->inverse_transform, inverse, mesh->inverse_transform);
}

void rotate_mesh_z_degrees(Mesh* mesh, double degrees){
    double transform[4][4];
    double inverse[4][4];
    double rads = to_radians(degrees);

    M3d_make_z_rotation_cs(transform, cos(degrees), sin(degrees));
    M3d_make_z_rotation_cs(transform, cos(degrees), -sin(degrees));

    M3d_mat_mult(mesh->transform, transform, mesh->transform);
    M3d_mat_mult(mesh->inverse_transform, inverse, mesh->inverse_transform);
}

/**
 * @brief Computes and sets the mesh's bounding box
 * 
 * @param mesh A pointer to the mesh whose bounding box will be computed
 */
void compute_mesh_bounds(Mesh* mesh){
    Vector3 min = {INFINITY, INFINITY, INFINITY};
    Vector3 max = {-INFINITY, -INFINITY, -INFINITY};

    for(int i = 0; i < mesh->num_vertices; i++){
        Vector3 position = mesh->vertices[i].position;
        
        if(position.x < min.x) min.x = position.x;
        if(position.y < min.y) min.y = position.y;
        if(position.z < min.z) min.z = position.z;

        if(position.x > max.x) max.x = position.x;
        if(position.y > max.y) max.y = position.y;
        if(position.z > max.z) max.z = position.z;
    }

    mesh->bounding_box_max = max;
    mesh->bounding_box_min = min;
}


void compute_face_normals(Mesh* mesh){
    for(int i = 0; i < mesh->num_tris; i++){
        Triangle tri = mesh->tris[i];
        Vector3 edge_1 = vec3_sub(tri.b->position, tri.a->position);
        Vector3 edge_2 = vec3_sub(tri.c->position, tri.a->position);

        mesh->tris[i].normal = vec3_normalized(vec3_cross_prod(edge_1, edge_2));
    }
}

//This doesn't account for clipping but It doesnt really matters
void debug_draw_mesh(Mesh mesh, Camera cam, int width, int height){
    for(int i = 0; i < mesh.num_tris; i++){
        Triangle tri = mesh.tris[i];
        Vector2 a, b, c;

        point_to_window(&a, tri.a->position, cam, width, height);
        point_to_window(&b, tri.b->position, cam, width, height);
        point_to_window(&c, tri.c->position, cam, width, height);
        
        Vector3 triangle_center = vec3_scale(vec3_add(vec3_add(tri.a->position, tri.b->position), tri.c->position), 1.0 / 3);
        if(vec3_distance(triangle_center, cam.eye) > cam.focal_length){
            G_rgb(SPREAD_COL3(vec3_scale(mesh.material.base_color, 0.5)));
        }
        else {
            G_rgb(SPREAD_COL3(mesh.material.base_color));
        }
        G_triangle(SPREAD_VEC2(a), SPREAD_VEC2(b), SPREAD_VEC2(c));
    }
}