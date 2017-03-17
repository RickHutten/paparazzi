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

#include "modules/computer_vision/lib/vision/image.h"

struct video_listener *listener = NULL;


// Filter Settings
uint8_t color_lum_min = 105;
uint8_t color_lum_max = 205;
uint8_t color_cb_min  = 52;
uint8_t color_cb_max  = 140;
uint8_t color_cr_min  = 180;
uint8_t color_cr_max  = 255;

// Result
int color_count = 0;

// Function
struct image_t *colorfilter_func(struct image_t *img) {
	// Filter
	//  color_count = image_yuv422_colorfilt(img, img,
	//                                       color_lum_min, color_lum_max,
	//                                       color_cb_min, color_cb_max,
	//                                       color_cr_min, color_cr_max
	//                                      );
	color_count = process_image(img, img,
								color_lum_min, color_lum_max,
								color_cb_min, color_cb_max,
								color_cr_min, color_cr_max
	);

	return img; // Colorfilter did not make a new image
}

void colorfilter_init(void) {
	listener = cv_add_to_device(&COLORFILTER_CAMERA, colorfilter_func);
}


uint16_t process_image(struct image_t *input, struct image_t *output, uint8_t y_m, uint8_t y_M, uint8_t u_m,
		uint8_t u_M, uint8_t v_m, uint8_t v_M) {
	uint16_t cnt = 0;
	uint8_t *source = input->buf;
	uint8_t *dest = output->buf;

	// Copy the creation timestamp (stays the same)
	output->ts = input->ts;
	// Go trough all the pixels
	int image_height = output->w;
	int image_width = output->h;

	int box_width = 20;
	int box_height = 20;
	int box_surface = box_width * box_height;

	int boxes[image_height / box_height][image_width / box_width];
	int boundary[image_width / box_width];
	printf("boxes: %d, %d\n", image_height / box_height, image_width / box_width);


	for (uint16_t x = 0; x < output->h; x++) {
		for (uint16_t y = 0; y < output->w; y += 2) {
			if (is_grass(dest[1], dest[0], dest[2])) {
				// Als de pixel groen is
//				dest[0] = 255;      // U
//				dest[1] = 29;  		// Y
//				dest[2] = 107;		// V
//				dest[3] = source[3];

				boxes[(y / (box_height))][(x / (box_width))] += 1;
			}
			// Go to the next 2 pixels
			dest += 4;
			source += 4;
		}
	}

	float fraction = 0.85;
	for (int i = 0; i < image_height / box_height; i++) {
		for (int j = (image_width / box_width - 1); j > -1; j--) {
			if (boxes[i][j] < fraction * box_surface) {
				boundary[j] = i * box_height;
			}
		}
	}

	printf("boundary[12]: %d\n", boundary[12]);

	// Draw limit
	for (uint16_t x = 0; x < output->h; x++) {
		for (uint16_t y = 0; y < output->w; y += 2) {
			if (y == boxes[(y / box_height)][(x / box_width)]) {
				dest[0] = 255;      // U
				dest[1] = 29;  		// Y
				dest[2] = 107;		// V
				dest[3] = source[3];
			}

			// Go to the next 2 pixels
			dest += 4;
			source += 4;
		}
	}


	printf("einde\n");
	return 0;
}

int is_grass(int y, int u, int v) {
	if (y > 50 && y < 170) {
		if ((u > 30 && u < 110)) {
			if ((v > 120 && v < 160)) {
			return 1;
			}
		}
	}
	return 0;
}




