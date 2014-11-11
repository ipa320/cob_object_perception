/*
 * texture_features.h
 *
 *  Created on: 13.01.2014
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef AMADASUN_H_
#define AMADASUN_H_

//#include <cob_texture_categorization/texture_categorization.h>
#include "cob_texture_categorization/texture_features.h"


class amadasun
{
public:
	amadasun();
	void get_amadasun(const cv::Mat& img, double d, struct feature_results *results, double& contrast_raw);

};
#endif /* AMADASUN_H_ */
