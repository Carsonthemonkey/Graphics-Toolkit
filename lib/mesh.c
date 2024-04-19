#include <math.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include "mesh.h"
#include "M3d_matrix_tools.h"
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
    while(fgets(line, BUFFER_SIZE, f) && i < mesh->num_vertices){
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
    while(fgets(line, BUFFER_SIZE, f) && i < mesh->num_tris){

        token = strtok(line, "\n ");
        if(!strcmp(token, "comment")) continue;

        mesh->tris[i].a = atoi(token);
        token = strtok(NULL, "\n ");
        mesh->tris[i].b = atoi(token);
        token = strtok(NULL, "\n ");
        mesh->tris[i].c = atoi(token);
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
