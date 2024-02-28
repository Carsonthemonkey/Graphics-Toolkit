#include <stdio.h>
#include <stdlib.h>
#include <png.h>
#include "texture.h"
#include "xwd_tools.h"

const Texture NULL_TEXTURE = {0, 0, 0, {-1}};

Texture new_xwd_texture(char* filename){
    Texture result;
    result.type = XWD;
    result.data.xwd_texture_id = init_xwd_map_from_file(filename);
    if(result.data.xwd_texture_id == -1){
        fprintf(stderr, "Failed to load XWD file\n");
        exit(1);
    }
    int dimensions[2];
    int status = get_xwd_map_dimensions(result.data.xwd_texture_id, dimensions);
    if(status == -1){
        fprintf(stderr, "Failed to get texture dimensions from file\n");
        exit(1);
    }
    result.width = dimensions[0];
    result.height = dimensions[1];
    return result;
}

//TODO: support transparency (alpha)
Color3 get_texture_color(Texture texture, Vector2 position){
    Color3 result;
    if(texture.type == PNG){
        if(texture.data.row_pointers == NULL){
            fprintf(stderr, "Error sampling PNG texture. Seems to not be initialized\n");
            exit(1);
        }
        png_bytep row = texture.data.row_pointers[(int)position.y];
        png_bytep pixel = &(row[(int)position.x * 4]);
        result.r = pixel[0] / 255.0;
        result.g = pixel[1] / 255.0;
        result.b = pixel[2] / 255.0;
        return result;
    }
    else{
        if(position.x > texture.width || position.y > texture.height || position.x < 0 || position.y < 0){
            fprintf(stderr, "Texture sample out of bounds.\n");

            exit(1);
        }
        get_xwd_map_color(texture.data.xwd_texture_id, (int)position.x, (int)position.y, (double *)&result);
        return result;
    }
}

//* Implementation from https://gist.github.com/niw/5963798
Texture new_png_texture(char* filename){
    Texture result;
    result.type = PNG;
    result.data.row_pointers = NULL;
    FILE *fp = fopen(filename, "rb");
    if(fp == NULL){
        fprintf(stderr, "Could not find file '%s'", filename);
        exit(1);
    }
    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if(!png){ 
        fprintf(stderr, "error loading png file\n");
        exit(1);
    }
    png_infop info = png_create_info_struct(png);
    if(!info){ 
        fprintf(stderr, "error loading png info\n");
        exit(1);
    }
    if(setjmp(png_jmpbuf(png))) abort();
    
    png_init_io(png, fp);

    png_read_info(png, info);

    result.width = png_get_image_width(png, info);
    result.height = png_get_image_height(png, info);
    // printf("width, height: %i, %i\n", result.width, result.height);
    png_byte color_type = png_get_color_type(png, info);
    png_byte bit_depth = png_get_bit_depth(png, info);

    if(bit_depth == 16){
        png_set_strip_16(png);
    }

    if(color_type == PNG_COLOR_TYPE_PALETTE) png_set_palette_to_rgb(png);
    if(png_get_valid(png, info, PNG_INFO_tRNS)) png_set_tRNS_to_alpha(png);

    if(color_type == PNG_COLOR_TYPE_RGB ||
     color_type == PNG_COLOR_TYPE_GRAY ||
     color_type == PNG_COLOR_TYPE_PALETTE)
    png_set_filler(png, 0xFF, PNG_FILLER_AFTER);

    if(color_type == PNG_COLOR_TYPE_GRAY ||
     color_type == PNG_COLOR_TYPE_GRAY_ALPHA)
    png_set_gray_to_rgb(png);

    png_read_update_info(png, info);

    if(result.data.row_pointers) {
        fprintf(stderr, "Error loading PNG, row pointers are not null\n");
        exit(1);
    } 

    result.data.row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * result.height);
    for(int y = 0; y < result.height; y++){
        result.data.row_pointers[y] = (png_byte*)malloc(png_get_rowbytes(png, info));
    }
    

    png_read_image(png, result.data.row_pointers);

    fclose(fp);

    png_destroy_read_struct(&png, &info, NULL);
    return result;
}

bool texture_is_null(Texture texture){
    return (texture.width <= 0 || texture.height <= 0);
}