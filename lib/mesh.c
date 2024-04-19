#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include "mesh.h"

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
Mesh load_mesh_from_ply(char* filename){
    // Read until element

    // Record element number and type
    // check if there are normals or assert there are
    // repeat until end_header
    // malloc space for vertices and tris
    // Read in points
    Mesh result;
    result.num_vertices = -1;
    result.num_tris = -1;
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
    while(fgets(line, BUFFER_SIZE, f) && (result.num_vertices ==  -1 || result.num_tris == -1)){
        token = strtok(line, " ");
        // Find the number of faces and vertices
        
        if(strcmp(token, "element")) continue;
        else{
            char* element_type = strtok(NULL, " ");
            int element_num = atoi(strtok(NULL, " "));
            if(!strcmp(element_type, "vertex")){
                result.num_vertices = element_num;
            }
            else if(!strcmp(element_type, "face")){
                result.num_tris = element_num;
            }
            else {
                fprintf(stderr, "Unknown element type '%s'\n", element_type);
                exit(1);
            }
        }
    }

    // Face num and vertex num have been recorded
    result.num_tris = result.num_tris;
    result.num_vertices = result.num_vertices; 
    result.vertices = (Vertex*)malloc(sizeof(Vertex) * result.num_vertices);
    if(result.vertices == NULL) goto MEM_ERROR;
    result.tris = (Triangle*)malloc(sizeof(Triangle) * result.num_tris);
    if(result.tris == NULL) goto MEM_ERROR;

    while(fgets(line, BUFFER_SIZE, f)){
        token = strtok(line, " ");
        trim_trailing_whitespace(token);
        if(!strcmp(token, "end_header")) break;
    }
    // printf("End header\n");
    // Now we read values
    int i = 0;
    while(fgets(line, BUFFER_SIZE, f) && i < result.num_vertices){
        // Read positions
        token = strtok(line, "\n\r\t ");
        if(!strcmp(token, "comment")) continue;
        result.vertices[i].position.x = strtod(token, NULL);
        token = strtok(NULL, "\n\r\t ");
        result.vertices[i].position.y = strtod(token, NULL);
        token = strtok(NULL, "\n\r\t ");
        result.vertices[i].position.z = strtod(token, NULL);
        
        //Read normals
        token = strtok(NULL, "\n\r\t ");
        result.vertices[i].normal.x = strtod(token, NULL);
        token = strtok(NULL, "\n\r\t ");
        result.vertices[i].normal.y = strtod(token, NULL);
        token = strtok(NULL, "\n\r\t ");
        result.vertices[i].normal.z = strtod(token, NULL);
        i++;
    }
    i = 0;
    while(fgets(line, BUFFER_SIZE, f) && i < result.num_tris){

        token = strtok(line, "\n ");
        if(!strcmp(token, "comment")) continue;

        result.tris[i].a = atoi(token);
        token = strtok(NULL, "\n ");
        result.tris[i].b = atoi(token);
        token = strtok(NULL, "\n ");
        result.tris[i].c = atoi(token);
        i++;
    }


    fclose(f);
    return result;

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