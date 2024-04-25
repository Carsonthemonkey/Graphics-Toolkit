#include "buffer.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "colors.h"
#include "FPToolkit.h"

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

void copy_image_buffer(Color3* source, Color3* dest, int width, int height){
    memcpy(source, dest, sizeof(Color3) * width * height);
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

void copy_float_image_buffer(Color3f* source, Color3f* dest, int width, int height){
    memcpy(dest, source, sizeof(Color3f) * width * height);
}

void delete_float_image_buffer(Color3f* buffer){
    free(buffer);
}

void copy_image_to_float_image(Color3* source, Color3f* dest, int width, int height){
    for(int y = 0; y < height; y++){
        for(int x = 0; x < width; x++){
            Color3 pixel_color = get_image_buffer_pixel(source, x, y, width);
            Color3f float_pixel_color = vec3_to_vec3f(pixel_color);
            set_float_image_buffer_pixel(dest, float_pixel_color, x, y, width);
        }
    }
}

void apply_pixel_filter_to_float_image(Color3f* buffer,
                                       Color3f (*filter)(Color3f),
                                       int width, int height){
    for(int y = 0; y < height; y++){
        for(int x = 0; x < width; x++){
            Color3f current = get_float_image_buffer_pixel(buffer, x, y, width);
            Color3f new = filter(current);
            set_float_image_buffer_pixel(buffer, new, x, y, width);
        }
    }
}

void draw_float_buffer(Color3f* buffer, int width, int height){
    for(int y = 0; y < height; y++){
        for(int x = 0; x < width; x++){
            Color3f pixel_color = get_float_image_buffer_pixel(buffer, x, y, width);
            G_rgb(SPREAD_COL3(pixel_color));
            G_pixel(x, y);
        }
    }
}