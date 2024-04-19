#ifndef FPTOOLKIT_H
#define FPTOOLKIT_H

// Initializes the graphics system with the specified width and height.

int G_init_graphics(double width, double height);

int G_wait_key();

int G_no_wait_key();

extern int (*G_rgb)(double r, double g, double b);

extern int (*G_pixel)(double x, double y);

extern int (* G_get_pixel) (double x, double y) ;

extern int (*G_line)(double start_x, double start_y, double end_x, double end_y);

extern int (*G_display_image)();

extern int (*G_clear)();

extern int (*G_circle)(double x, double y, double radius);

extern int (*G_fill_circle)(double x, double y, double radius);

int (* G_triangle) (double x0, double y0, double x1, double y1, double x2, double y2) ; 

extern int (*G_point)(double x, double y);

// Saves the current image to a BMP file.
extern int (*G_save_to_bmp_file)(char *filename);

extern int (*G_save_image_to_file)(char *filename);

// Draws a string at the specified location.
int (*G_draw_string)(const void *text, double x, double y);

/**
int G_init_graphics(double width, double height);

// Closes the graphics system.
int (*G_close)();

// Clears the graphics window.
int (*G_clear)(void);

// Displays the current image.
int G_display_image(void);

// Waits for and returns a mouse click.
int (*G_wait_click)(double p[2]);

// Waits for and returns a key press.
int (*G_wait_key)(void);

// If key has been hit, return non-negative (ascii) value.
// Return negative if key has NOT been hit (some other action or no action).
int (*G_no_wait_key)(void);

// Returns the x coordinate of the last mouse event.
double (*G_x_mouse)(void);

// Returns the y coordinate of the last mouse event.
double (*G_y_mouse)(void);

// Sets the current drawing color using RGB values.
int (*G_rgb)(double r, double g, double b);


// Draws a line between two points.
int (*G_line)(double x_start, double y_start, double x_end, double y_end);

// Draws a circle with the specified center and radius.
int (*G_circle)(double x_center, double y_center, double radius);

// Fills a circle with the specified center and radius.
int (*G_fill_circle)(double x_center, double y_center, double radius);

// Draws a rectangle with the specified corner and dimensions.
int (*G_rectangle)(double x_left, double y_top, double width, double height);

// Fills a rectangle with the specified corner and dimensions.
int (*G_fill_rectangle)(double x_left, double y_top, double width, double height);


// Fills a polygon defined by arrays of x and y coordinates and the number of points.
int (*G_fill_polygon)(double *xx, double *yy, double n);

// Draws a polygon defined by arrays of x and y coordinates and the number of points.
int (*G_polygon)(double *x, double *y, double numpts);


// Displays an image from a BMP file at the specified location.
int (*G_display_bmp_file)(char filename[], int xoffset, int yoffset);
*/
#endif // FPTOOLKIT_H
