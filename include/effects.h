#ifndef EFFECTS_H
#define EFFECTS_H
/**
 * @brief Draws edges around objects based on depth falloff
 * 
 * @param width The width of the screen
 * @param height The height of the screen
 * @param z_buffer The z-buffer of the current frame
 * @param threshold The depth threshold to determine if an edge should be drawn
 */
void depth_edge_effect(int width, int height, double z_buffer[width][height], double threshold);

#endif