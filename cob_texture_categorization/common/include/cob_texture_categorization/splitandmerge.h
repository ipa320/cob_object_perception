/*
 * splitandmerge.h
 *
 *  Created on: 02.07.2013
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef SPLITANDMERGE_H
#define SPLITANDMERGE_H

#include "create_lbp.h"
#include "get_mapping.h"
#include <string>

class splitandmerge
{
public:
	splitandmerge();
	cv::Mat categorize(cv::Mat image_in,  std::vector<cv::Mat>* segments, double mergeval);

};
#endif
