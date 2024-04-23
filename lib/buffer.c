#include "buffer.h"
#include <stdlib.h>
#include <stdio.h>
#include "colors.h"

Color3* create_image_buffer(int width, int height){
    Color3* buffer = (Color3*)malloc(sizeof(Color3) * width * height);
    if(buffer == NULL){
        fprintf(stderr, "Error: Failed to allocate sufficient memory for image buffer");
    }
    return buffer;
}

Color3 get_image_buffer_pixel(Color3* buffer, int x, int y, int width){
    return buffer[(y * width) + x];
}

void set_image_buffer_pixel(Color3* buffer, Color3 pixel, int x, int y, int width){
    buffer[(y * width) + x] = pixel;
}

void delete_image_buffer(Color3* buffer){
    free(buffer);
}

Color3f* create_float_image_buffer(int width, int height){
    Color3f* buffer = (Color3f*)malloc(sizeof(Color3f) * width * height);
    if(buffer == NULL){
        fprintf(stderr, "Error: Failed to allocate sufficient memory for image buffer");
    }
    return buffer;
}

Color3f get_float_image_buffer_pixel(Color3f* buffer, int x, int y, int width){
    return buffer[(y * width) + x];
}

void set_float_image_buffer_pixel(Color3f* buffer, Color3f pixel, int x, int y, int width){
    buffer[(y * width) + x] = pixel;
}

void delete_float_image_buffer(Color3f* buffer){
    free(buffer);
}