/**
 * @file colors.h
 * @author Carson Reader
 * @brief Some simple macros for commonly used RGB values
 * @version 0.1
 * @date 2024-01-23
 * 
 */
#include "vector.h"

/**
 * @brief An alias for Vector3. Behaves the exact same but is more semantically meaningful
 * 
 */
typedef Vector3 Color3;

/**
 * @brief An alias for Vector3f. Behaves the exact same but is more semantically meaningful
 * 
 */
typedef Vector3f Color3f;

#define SPREAD_COL3(c) (c).r, (c).g, (c).b

/**
 * @brief Transforms a linear color to the sRGB color space
 * 
 * @param linear_color The linear color to transform
 * @return Color3 The color transformed to sRGB
 */
Color3 linear_to_srgb(Color3 linear_color);

/**
 * @brief Transforms a color in the sRGB color space to linear
 * 
 * @param srgb_color The sRGB color
 * @return Color3 The color transformed to linear
 */
Color3 srgb_to_linear(Color3 srgb_color);

/**
 * @brief Applies tonemapping based on The Academy Color Encoding System (ACES)
 * 
 * @param color The color to tonemap
 * @return Color3 The tonemapped color
 */
Color3 aces_tonemap(Color3 color);

/**
 * @brief Transforms a linear color to the sRGB color space
 * 
 * @param linear_color The linear color to transform
 * @return Color3 The color transformed to sRGB
 */
Color3f linear_to_srgbf(Color3f linear_color);

/**
 * @brief Transforms a color in the sRGB color space to linear
 * 
 * @param srgb_color The sRGB color
 * @return Color3f The color transformed to linear
 */
Color3f srgb_to_linearft(Color3f srgb_color);

/**
 * @brief Applies tonemapping based on The Academy Color Encoding System (ACES)
 * 
 * @param color The color to tonemap
 * @return Color3f The tonemapped color
 */
Color3f aces_tonemapf(Color3f color);

#ifndef COLORS_H
#define COLORS_H

#define BLACK 0, 0, 0
#define WHITE 1, 1, 1
#define GRAY 0.5, 0.5, 0.5

#define RED 1, 0, 0
#define GREEN 0, 1, 0
#define BLUE 0, 0, 1

#define YELLOW 1, 1, 0
#define MAGENTA 1, 0, 1
#define CYAN 0, 1, 1

#endif