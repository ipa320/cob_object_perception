/*
 * lbp.h
 *
 *  Created on: 02.07.2013
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef LBP_H
#define LBP_H

#include "create_lbp.h"
#include "get_mapping.h"
#include <string>

class lbp
{
public:
	lbp();
	void lbp_compute(cv::Mat image, int radius, int number, struct mapping *mapping, std::string mode, int *table, float *hist);

};
#endif
