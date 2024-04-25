#ifndef DENOISE_H
#define DENOISE_H
#include "path_trace.h"

/**
 * @brief Initializes denoiser for a given scene
 * 
 * @param scene The scene to initialize the denoiser for
 */
void init_denoiser(PathTracedScene* scene);

/**
 * @brief Denoises a scene using it's image buffer. Requires denoiser to have been initialized using `init_denoiser`
 * 
 * @param scene The scene to denoise
 */
void denoise_image(PathTracedScene scene);

/**
 * @brief Frees all denoiser resources
 * 
 * @param scene The scene for which the denoiser was initialized
 */
void cleanup_denoiser(PathTracedScene* scene);

#endif