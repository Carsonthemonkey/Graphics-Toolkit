#ifndef BUFFER_H
#define BUFFER_H

#include "colors.h"

/**
 * @brief Create a new image buffer.
 * 
 * @param width The width of the buffer
 * @param height The height of the buffer
 * @return Color3* A pointer to the image buffer
 */
Color3* create_image_buffer(int width, int height);

/**
 * @brief Gets a pixel at a given location in an image buffer
 * 
 * @param buffer The image buffer to sample
 * @param x The x-coordinate of the pixel
 * @param y The y-coordinate of the pixel
 * @param width The width of the image buffer
 * @return Color3 The color of the pixel at the given coordinates
 */
Color3 get_image_buffer_pixel(Color3* buffer, int x, int y, int width);

/**
 * @brief Sets a pixel of an image buffer to a given color
 * 
 * @param buffer The buffer to modify
 * @param pixel The value to set the pixel to
 * @param x The x-coordinate of the pixel to set
 * @param y The y-coordinate of the pixel to set
 * @param width The width of the image buffer
 */
void set_image_buffer_pixel(Color3* buffer, Color3 pixel, int x, int y, int width);

/**
 * @brief Deletes a buffer from memory
 * 
 * @param buffer The buffer to delete
 */
void delete_image_buffer(Color3* buffer);

/**
 * @brief Create a new floating point image buffer. 
 * 
 * @param width The width of the image buffer
 * @param height The height of the image buffer
 * @return Color3f* A pointer to the image buffer
 */
Color3f* create_float_image_buffer(int width, int height);

/**
 * @brief Gets a pixel at a given location in an image buffer
 * 
 * @param buffer The buffer to sample
 * @param x The x-coordinate of the pixel
 * @param y The y-coordinate of the pixel
 * @param width The width of the buffer
 * @return Color3f The color of the pixel at the given coordinates
 */
Color3f get_float_image_buffer_pixel(Color3f* buffer, int x, int y, int width);

/**
 * @brief Sets a pixel of an image buffer to a given color
 * 
 * @param buffer The buffer to modify
 * @param pixel The value to set the pixel to
 * @param x The x-coordinate of the pixel to set
 * @param y The y-coordinate of the pixel to set
 * @param width The width of the image buffer
 */
void set_float_image_buffer_pixel(Color3f* buffer, Color3f pixel_color, int x, int y, int width);

/**
 * @brief Deletes a buffer from memory
 * 
 * @param buffer The buffer to delete
 */
void delete_float_image_buffer(Color3f* buffer);

#endif