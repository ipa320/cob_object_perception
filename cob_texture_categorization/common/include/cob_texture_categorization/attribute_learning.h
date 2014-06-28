/*
 * attribute_learning.h
 *
 *  Created on: 28.06.2014
 *      Author: Richard Bormann
 */

#ifndef ATTRIBUTE_LEARNING_H_
#define ATTRIBUTE_LEARNING_H_

#include "cob_texture_categorization/create_train_data.h"

#include <cv.h>


class AttributeLearning
{
public:
	AttributeLearning() {};

	void loadTextureDatabaseBaseFeatures(std::string filename, cv::Mat& feature_matrix, cv::Mat& attribute_matrix, create_train_data::DataHierarchyType& data_sample_hierarchy);

	// int cross_validation_mode: 0=leave out one object per class, 1=leave out one class
	void crossValidation(unsigned int folds, const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy, int cross_validation_mode);
};

#endif /* ATTRIBUTE_LEARNING_H_ */
