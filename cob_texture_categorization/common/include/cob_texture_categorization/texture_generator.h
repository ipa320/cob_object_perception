/*
 * object_recording.h
 *
 *  Created on: 30.10.2014
 *      Author: rbormann
 */

#ifndef TEXTURE_GENERATOR_H
#define TEXTURE_GENERATOR_H

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "cob_texture_categorization/texture_features.h"
#include "cob_texture_categorization/color_parameter.h"


enum ColorCode {BROWN=1, RED=2, ORANGE=3, YELLOW=4, GREEN=5, CYAN=6, BLUE=7, PURPLE=8, PINK=9, WHITE=10, GRAY=11, BLACK=12};

struct ColorData
{
	ColorCode color_code;
	std::string color_name;
	cv::Vec3b color_hsv;
};

class TextureGenerator
{
public:
	TextureGenerator();

	void generate_textures();

private:

	// generates a random primitive mask
	cv::Mat random_primitive_mask(int max_width, int max_height);

	// generates a random color in HSV color space
	ColorData random_color();
};

#endif // TEXTURE_GENERATOR_H
