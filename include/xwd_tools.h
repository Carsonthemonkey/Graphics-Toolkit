#ifndef XWD_TOOLS_H
#define XWD_TOOLS_H

#include <stdio.h>

/**
 * @brief Create a new xwd map.
 * 
 * @param map_width The width of the xwd map
 * @param map_height The height of the xwd map
 * @return int a return code indicating potential failure (-1)
 */
int create_new_xwd_map (int map_width, int map_height);

/**
 * @brief Initializes an XWD map from a file
 * 
 * @param filename The filename of the xwd file to read
 * @return int an ID referencing the xwd map to be used in other functions
 */
int init_xwd_map_from_file(char *filename);

/**
 * @brief Get the dimensions of an xwd map by ID
 * 
 * 
 * @param xwd_id_number The id number of the XWD map
 * @param dimensions_out A buffer that the dimensions will be written to.
 * @return int a return code indicating potential failure (-1)
 */
int get_xwd_map_dimensions(int xwd_id_number, int dimensions_out[2]);

/**
 * @brief Get the the color of a given pixel in an XWD map
 * 
 * @param xwd_id_number The id number of the xwd map to sample
 * @param x The x coordinate of the pixel to sample
 * @param y The y coordinate of the pixel to sample
 * @param rgb_out A buffer that the rgb color of the pixel will be written to
 * @return int a return code indicating a potential failure (-1)
 */
int get_xwd_map_color(int xwd_id_number, int x, int y, double rgb_out[3]);


/**
 * @brief set the the color of a given pixel in an XWD map
 * 
 * @param xwd_id_number The id number of the xwd map to set.
 * @param x The x-coordinate of the pixel.
 * @param y The y-coordinate of the pixel.
 * @param r The red component 
 * @param g The green component 
 * @param b The blue component 
 * @return int return code indicating a potential failure (-1)
 */
int set_xwd_map_color(int xwd_id_numner, int x, int y, double r, double g, double b);

/**
 * @brief Sets all pixels in the xwd map to the provided color
 * 
 * @param xwd_id_marker The id number of the xwd map to clear.
 * @param r The red component
 * @param g The green component
 * @param b The blue component
 * @return int return code indicating a potential failure (-1)
 */
int clear_xwd_map(int xwd_id_marker, double r, double g, double b);

/**
 * @brief 
 * 
 * @param mapid The id number of the xwd map to save.
 * @param fp The file to write the result to.
 * @return int return code indicating a potential failure (-1)
 */
int xwd_map_to_xwd_file (int mapid, FILE *fp);

/**
 * @brief Saves the XWD map to a named file
 * 
 * @param mapid The id number of the xwd map to save.
 * @param fname The name of the file to write to
 * @return int return code indicating a potential failure (-1)
 */
int xwd_map_to_named_xwd_file (int mapid, char *fname);


#endif  