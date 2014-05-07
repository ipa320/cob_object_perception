/*
 * run_meanshift_test.h
 *
 *  Created on: 02.07.2013
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef RUN_MEANSHIFT_TEST_H
#define RUN_MEANSHIFT_TEST_H

#include "create_lbp.h"
#include "get_mapping.h"
#include <string>

class run_meanshift_test
{
public:
	run_meanshift_test();
	void run_test(cv::Mat* image, cv::Mat depth, std::vector < std::vector<cv::Mat> >* regions);

};
#endif
