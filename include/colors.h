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

#define SPREAD_COL3(c) (c).r, (c).g, (c).b

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