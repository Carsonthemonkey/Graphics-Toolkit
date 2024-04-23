#include "denoise.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h> 
#include <OpenImageDenoise/oidn.h>
#include "path_trace.h"
#include "colors.h"

bool initialized = false;
OIDNDevice device;
OIDNFilter filter;
OIDNBuffer color_buffer;

void init_denoiser(PathTracedScene* scene){
    if(initialized) {
        fprintf(stderr, "Error: Attempted to initialize denoiser more than once\n");
        return;
    }
    initialized = true;
    device = oidnNewDevice(OIDN_DEVICE_TYPE_CPU);
    oidnCommitDevice(device);
    int num_pixels = scene->width * scene->height;
    color_buffer = oidnNewBuffer(device, num_pixels * sizeof(float) * 3);
    scene->denoise_buffer = (float*)oidnGetBufferData(color_buffer);
    filter = oidnNewFilter(device, "RT");
    oidnSetFilterImage(filter, "color", color_buffer, OIDN_FORMAT_FLOAT3, scene->width, scene->height, 0, 0, 0);
    //TODO: Maybe use another buffer for denoise output?
    oidnSetFilterImage(filter, "output", color_buffer, OIDN_FORMAT_FLOAT3, scene->width, scene->height, 0, 0, 0);
    oidnSetFilterBool(filter, "hdr", true);
    oidnCommitFilter(filter);
}

void fill_denoise_buffer(Color3* image_buffer, float* denoise_buffer, int width, int height){
    int num_pixels = width * height;
    for(int p = 0; p < num_pixels; p++){
        int y = p / width;
        int x = p - (y * width);
        Color3 image_pixel = get_pixel(image_buffer, width, x, y);
        denoise_buffer[p * 3] = (float)image_pixel.r;
        denoise_buffer[(p * 3) + 1] = (float)image_pixel.g;
        denoise_buffer[(p * 3) + 2] = (float)image_pixel.b;
    }
}

void denoise_image(PathTracedScene scene){
    fill_denoise_buffer(scene.screen_buffer, scene.denoise_buffer, scene.width, scene.height);
    oidnExecuteFilter(filter);

    const char* errorMessage;
    if (oidnGetDeviceError(device, &errorMessage) != OIDN_ERROR_NONE)
    fprintf(stderr, "OpenImageDenoise Error: %s\n", errorMessage);
}

void cleanup_denoiser(PathTracedScene* scene){
    if(!initialized){
        fprintf(stderr, "Error: Attempted to cleanup denoiser, but no denoiser was initialized\n");
        return;
    }
    initialized = false;
    scene->denoise_buffer = NULL;
    oidnReleaseBuffer(color_buffer);
    oidnReleaseFilter(filter);
    oidnReleaseDevice(device);
}