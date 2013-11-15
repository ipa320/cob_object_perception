/*
 * categorization_fkt.h
 *
 *  Created on: 11.10.2013
 *      Author: rbormann, Daniel Hundsdoerfer
 */


#ifndef CREATE_LBP_H
#define CREATE_LBP_H


#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


	struct mapping {
			int samples;
			int num;
		};

class create_lbp{
public:
	create_lbp();
	void create_lbp_class(cv::Mat image_in, int radius, int samples, bool ff,double *lbp_hist);



};


#endif
