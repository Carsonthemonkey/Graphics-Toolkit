#include <math.h>
#include "colors.h"

// Credit to https://www.nayuki.io/page/srgb-transform-library
double linear_to_srgb_double(double x){
	if (x <= 0.0)
		return 0.0;
	else if (x >= 1.0)
		return 1.0;
	else if (x < 0.0031308)
		return x * 12.92;
	else
		return pow(x, 1.0 / 2.4) * 1.055 - 0.055;
}

double srgb_to_linear_double(double x) {
	if (x <= 0.0)
		return 0.0;
	else if (x >= 1.0)
		return 1.0;
	else if (x < 0.04045)
		return x / 12.92;
	else
		return pow((x + 0.055) / 1.055, 2.4);
}

Color3 linear_to_srgb(Color3 linear_color){
    return (Color3){
        linear_to_srgb_double(linear_color.x),
        linear_to_srgb_double(linear_color.y),
        linear_to_srgb_double(linear_color.z),
    };
}

Color3 srgb_to_linear(Color3 srgb_color){
    return (Color3){
        srgb_to_linear_double(srgb_color.x),
        srgb_to_linear_double(srgb_color.y),
        srgb_to_linear_double(srgb_color.z),
    };
}

double clamp(double val, double min, double max) {
    if (val < min) return min;
    else if (val > max) return max;
    else return val;
}
// Credit to https://blog.demofox.org/2020/06/06/casual-shadertoy-path-tracing-2-image-improvement-and-glossy-reflections/
double aces_tonemap_double(double x){
    double a = 2.51;
    double b = 0.03;
    double c = 2.43;
    double d = 0.59;
    double e = 0.14;
    return clamp((x*(a*x + b)) / (x*(c*x + d) + e), 0.0, 1.0);
}

Color3 aces_tonemap(Color3 color){
    return (Color3){
        aces_tonemap_double(color.x),
        aces_tonemap_double(color.y),
        aces_tonemap_double(color.z),
    };
}