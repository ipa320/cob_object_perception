/*
 * lbp.h
 *
 *  Created on: 02.07.2013
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef SVM_H
#define SVM_H

class svm
{
public:
	svm(cv::Mat image_in, int radius, int samples, bool hist_features);
};

struct mapping {
	int samples;
	int num;
};

#endif
