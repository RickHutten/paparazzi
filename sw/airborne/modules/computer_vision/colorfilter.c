/*
* Copyright (C) 2015
*
* This file is part of Paparazzi.
*
* Paparazzi is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2, or (at your option)
* any later version.
*
* Paparazzi is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Paparazzi; see the file COPYING.  If not, write to
* the Free Software Foundation, 59 Temple Place - Suite 330,
* Boston, MA 02111-1307, USA.
*/

/**
* @file modules/computer_vision/colorfilter.c
*/

// Own header
#include "modules/computer_vision/colorfilter.h"
#include <stdio.h>
#include <time.h>

#include "modules/computer_vision/lib/vision/image.h"

struct video_listener *listener = NULL;


// Filter Settings
uint8_t color_lum_min = 50;
uint8_t color_lum_max = 170;
uint8_t color_cb_min  = 0;
uint8_t color_cb_max  = 110;
uint8_t color_cr_min  = 0;
uint8_t color_cr_max  = 170;

// Result
int box_width = 10;
int box_height = 12;
int color_count = 0;
int boundary[520 / 10];
int boundary_smoothed[(520 / 10) - 2];
int threshold = 40;

// Function
struct image_t *colorfilter_func(struct image_t *img) {
	// Filter, changes boundary[] variable
//	clock_t begin = clock();
	process_image(img, img);
//	clock_t end = clock();
//	printf("Time per frame: %f\n", ((double)(end-begin))/CLOCKS_PER_SEC);

	return img; // Colorfilter did not make a new image
}

void colorfilter_init(void) {
	listener = cv_add_to_device(&COLORFILTER_CAMERA, colorfilter_func);
}


uint16_t process_image(struct image_t *input, struct image_t *output) {
	int cnt = 0;  // Counts number of steps the pointer takes throughout the loop
	uint8_t *source = input->buf;
	uint8_t *dest = output->buf;

	// Copy the creation timestamp (stays the same)
	output->ts = input->ts;
	// Go trough all the pixels
	int image_height = output->w;
	int image_width = output->h;

	// Area of the individual boxes
	int box_area = box_width * box_height / 2;

	// Initialize boxes that count green pixels per section of the image
	int boxes[image_height / box_height][image_width / box_width];
	memset(boxes, 0, sizeof boxes);  // Set all values to zero

	// Loop through image
	for (uint16_t x = 0; x < image_width; x++) {
		for (uint16_t y = 0; y < image_height; y += 2) {
			// Mark center pixel (for debugging)
			if (x == 270 && y == 120) {
				// Mark the pixel blue
//				printf("Color: (%d %d %d)   ", dest[1], dest[0], dest[2]);
				dest[0] = 0;        // U
				dest[1] = 29;  		  // Y
				dest[2] = 255;		  // V
			}
			// Check if pixel is grass
			if (is_grass(dest[1], dest[0], dest[2])) {
				// Mark the pixel blue
				dest[0] = 255;        // U
				dest[1] = 29;  		  // Y
				dest[2] = 107;		  // V

				// Increment count in the corresponding box
				boxes[(y / (box_height))][(x / (box_width))] += 1;
			}

			// Show the smoothed boundary on the image, the boundary
			// is 1 frame behind but it's only for visual purposes
			if (x > 2 * box_width && x < image_width - 2 * box_width &&
					(y == boundary_smoothed[x / box_width - 1] || y+1 == boundary_smoothed[x / box_width - 1])) {
				dest[0] = 128;      // U
				dest[1] = 250;  	// Y
				dest[2] = 20;		// V
			}

			// Draw the threshold on the image
			if (y == threshold || y + 1 == threshold) {
				dest[0] = 20;      // U
				dest[1] = 250;  	// Y
				dest[2] = 200;		// V
			}

			// Go to the next 2 pixels
			cnt += 4;
			dest += 4;
			source += 4;
		}
	}
	// Fraction of the box that needs to be grass
	float fraction = 0.80;

	// Loop through boxes from the bottom up
	for (int j = (image_width / box_width - 1); j > -1; j--) {  // Columns (left to right)
		for (int i = 0; i < image_height / box_height; i++) {  // Rows (bottom to top)
			// If box is not grass, save the value of the y coordinate
			if (boxes[i][j] < (fraction * box_area)) {
				boundary[j] = i * box_height;
				break;  // Go to next column
			} else if (i == image_height / box_height - 1) {
				// The column is green all the way up
				boundary[j] = image_height;
			}
		}
	}
	return 0;
}


/*
 * Color filter that returns 1 if the pixel is grass, and 0 if not
 */
int is_grass(int y, int u, int v) {
	if (y > color_lum_min && y < color_lum_max) {
		if ((u > color_cb_min && u < color_cb_max)) {
			if ((v > color_cr_min && v < color_cr_max)) {
				return 1;
			}
		}
	} else if (y > 30 && y <= color_lum_min) {
		if (u < 70 && v < 70 - u) {
			return 1;
		}
	}
	return 0;
}

