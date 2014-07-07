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

	void loadTextureDatabaseBaseFeatures(std::string filename, const int feature_number, const int attribute_number, cv::Mat& feature_matrix, cv::Mat& attribute_matrix, cv::Mat& class_label_matrix, create_train_data::DataHierarchyType& data_sample_hierarchy);

	void loadTextureDatabaseLabeledAttributeFeatures(std::string filename, cv::Mat& attribute_matrix, cv::Mat& class_label_matrix, create_train_data::DataHierarchyType& data_sample_hierarchy);

	// saves the training indices, computed attributes and class labels for each fold of an attribute learning cross-validation
	void saveAttributeCrossValidationData(std::string path, const std::vector< std::vector<int> >& preselected_train_indices, const std::vector<cv::Mat>& attribute_matrix_test_data, const std::vector<cv::Mat>& class_label_matrix_test_data);
	void loadAttributeCrossValidationData(std::string path, std::vector< std::vector<int> >& preselected_train_indices, std::vector<cv::Mat>& attribute_matrix_test_data, std::vector<cv::Mat>& class_label_matrix_test_data);

	// feature_matrix: matrix of features that should be mapped to the provided ground truth attributes
	// attribute matrix: ground truth attribute matrix
	// int cross_validation_mode: 0=leave out one object per class, 1=leave out one class
	// return_set_data: if true, all the training index sets, testing data and testing labels of all folds are returned
	void crossValidation(unsigned int folds, const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy, int cross_validation_mode);
	void crossValidation(unsigned int folds, const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy, int cross_validation_mode,
			bool return_set_data, const cv::Mat& class_label_matrix, std::vector< std::vector<int> >& preselected_train_indices, std::vector<cv::Mat>& attribute_matrix_test_data, std::vector<cv::Mat>& class_label_matrix_test_data);
};

#endif /* ATTRIBUTE_LEARNING_H_ */
