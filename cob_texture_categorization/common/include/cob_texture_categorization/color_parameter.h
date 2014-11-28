/*
 * texture_features.h
 *
 *  Created on: 13.01.2014
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef COLOR_PARAMETER_H_
#define COLOR_PARAMETER_H_

//#include <cob_texture_categorization/texture_categorization.h>
#include "cob_texture_categorization/texture_features.h"

class color_parameter
{
public:
	color_parameter();

	void get_color_parameter_new(cv::Mat img, struct feature_results *color_results, cv::Mat* raw_features=0);

	void get_color_parameter(cv::Mat img, struct feature_results *color_results, cv::Mat* raw_features=0);
};
#endif /* COLOR_PARAMETER_H_ */
