/*
 * train_svm.h
 *
 *  Created on: 13.01.2014
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef TRAIN_SVM_H_
#define TRAIN_SVM_H_

#include <cob_texture_categorization/texture_categorization.h>


class train_svm
{
public:
	train_svm();
	void run_training(std::string *trainingdata, std::string *traininglabel, double gam, double val, std::string *path);

};
#endif /* TRAIN_SVM_H_ */
