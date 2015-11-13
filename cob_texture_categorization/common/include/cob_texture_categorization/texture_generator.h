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

	// saves all generated data of one image
	void save_generated_data(const std::string path, const cv::Mat& image, const int image_counter, std::stringstream& annotation_data, const feature_results& attributes);

	// hsv -> bgr conversion and blurring
	void post_processing(const cv::Mat& image_hsv, cv::Mat& image_bgr, const double blur_factor);

	// put primitives into an image with specified arragement
	void place_primitives(cv::Mat& image, const std::vector<cv::Mat>& primitives, const std::vector<ColorData>& colors, const double directionality, const double primitive_number, int& dominant_color_index, int& dominant_color2_index);

	int count_non_zeros(const cv::Mat& image);

	// put colors on image
	void colorize_gray_image(cv::Mat& image, const std::vector<ColorData>& colors, int& dominant_color_index, int& dominant_color2_index);

	// creates number_primitves different primitive templates
	void distinct_random_primitives(std::vector<cv::Mat>& primitives, int number_primitves, int max_width, int max_height);

	// generates a random lined pattern
	// line_thickness_value, line_spacing_value: relative values in [0,1] representing the ratio of chosen value to possible range
	cv::Mat random_lined_pattern(int max_width, int max_height, double* line_thickness_value=0, double* line_spacing_value=0);

	// generates a random checked pattern
	// line_thickness_value, line_spacing_value: relative values in [0,1] representing the ratio of chosen value to possible range
	cv::Mat random_checked_pattern(int max_width, int max_height, double* line_thickness_value=0, double* line_spacing_value=0);

	// generates a random primitive mask
	cv::Mat random_primitive_mask(int max_width, int max_height);

	// randomly rotates an image
	void random_image_rotation(cv::Mat& image);

	// draws number_colors different random colors
	void distinct_random_colors(std::vector<ColorData>& colors, int number_colors);

	// generates a random color in HSV color space
	ColorData random_color();
};

#endif // TEXTURE_GENERATOR_H
