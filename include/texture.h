#ifndef TEXTURE_H
#define TEXTURE_H

#include <png.h>
#include <stdbool.h>

#include "colors.h"
#include "vector.h"

enum TextureFormat {
    XWD,
    PNG
};

/**
 * @brief A struct that contains data about a loaded texture
 * 
 */
typedef struct {
    // int id;
    int width;
    int height;
    enum TextureFormat type;
    union {
        int xwd_texture_id; //works for xwd files
        png_bytep *row_pointers; //for pngs 
    } data;
} Texture;

extern const Texture NULL_TEXTURE;

/**
 * @brief Loads a new texture from an XWD map and returns a Texture struct
 * 
 * @param filename The path of the file to load
 * @return Texture A struct representing the texture
 */
Texture new_xwd_texture(char* filename);

/**
 * @brief Get the color of a texture at the given x, y position
 * 
 * @param texture The texture to sample
 * @param position The pixel position on the texture to sample
 * @return Color3 The color of the sampled pixel
 */
Color3 get_texture_color(Texture texture, Vector2 position);

Texture new_png_texture(char* filename);

bool texture_is_null(Texture texture);

#endif