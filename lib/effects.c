#include <stdio.h>
#include "effects.h"
#include "colors.h"
#include "FPToolkit.h"
#include "math.h"

void depth_edge_effect(int width, int height, double z_buffer[width][height], double threshold){
    G_rgb(BLACK);
    for(int x = 1; x < width; x++){
        for(int y = 1; y < height; y++){
            double prev_depth = z_buffer[x][y - 1];
            double current_depth = z_buffer[x][y];

            if(fabs(current_depth - prev_depth) > threshold){
                G_pixel(x, y - 1);
                G_pixel(x, y);
            }
        }
    }
}

