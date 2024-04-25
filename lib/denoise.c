#include "denoise.h"
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h> 
#include <OpenImageDenoise/oidn.h>
#include <OpenImageDenoise/config.h>
#include "path_trace.h"
#include "colors.h"
#include "buffer.h"

bool initialized = false;
OIDNDevice device;
OIDNFilter filter;

OIDNBuffer color_buffer;
OIDNBuffer albedo_buffer;
OIDNBuffer normal_buffer;

void init_denoiser(PathTracedScene* scene){
    if(initialized) {
        fprintf(stderr, "Error: Attempted to initialize denoiser more than once\n");
        return;
    }
    initialized = true;
    device = oidnNewDevice(OIDN_DEVICE_TYPE_DEFAULT);
    const char* errorMessage;
    if (oidnGetDeviceError(device, &errorMessage) != OIDN_ERROR_NONE)
    fprintf(stderr, "OpenImageDenoise Error: %s\n", errorMessage);

    oidnCommitDevice(device);
    //TODO: print info about chosen denoising device
    int num_pixels = scene->width * scene->height;

    color_buffer = oidnNewBuffer(device, num_pixels * sizeof(float) * 3);
    scene->denoise_buffer = (Color3f*)oidnGetBufferData(color_buffer);

    albedo_buffer = oidnNewBuffer(device, num_pixels * sizeof(float) * 3);
    scene->albedo_buffer = (Color3f*)oidnGetBufferData(albedo_buffer);

    normal_buffer = oidnNewBuffer(device, num_pixels * sizeof(float) * 3);
    scene->normal_buffer = (Color3f*)oidnGetBufferData(normal_buffer);

    filter = oidnNewFilter(device, "RT");
    oidnSetFilterImage(filter, "color", color_buffer, OIDN_FORMAT_FLOAT3, scene->width, scene->height, 0, 0, 0);
    oidnSetFilterImage(filter, "albedo", albedo_buffer, OIDN_FORMAT_FLOAT3, scene->width, scene->height, 0, 0, 0);
    oidnSetFilterImage(filter, "normal", normal_buffer, OIDN_FORMAT_FLOAT3, scene->width, scene->height, 0, 0, 0);
    oidnSetFilterImage(filter, "output", color_buffer, OIDN_FORMAT_FLOAT3, scene->width, scene->height, 0, 0, 0);
    oidnSetFilterBool(filter, "hdr", true);
    oidnCommitFilter(filter);
}

void denoise_image(PathTracedScene scene){
    // fill_denoise_buffer(scene.light_buffer, scene.denoise_buffer, scene.width, scene.height);
    //TODO: This should probably not happen in this function
    copy_image_to_float_image(scene.light_buffer, scene.denoise_buffer, scene.width, scene.height);
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