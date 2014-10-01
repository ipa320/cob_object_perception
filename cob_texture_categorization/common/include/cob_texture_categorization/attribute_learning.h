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
	enum CrossValidationMode {LEAVE_OUT_ONE_OBJECT_PER_CLASS = 0, LEAVE_OUT_ONE_CLASS = 1};

	AttributeLearning() {};

	void loadTextureDatabaseBaseFeatures(std::string filename, const int feature_number, const int attribute_number, cv::Mat& feature_matrix, cv::Mat& ground_truth_attribute_matrix, cv::Mat& class_label_matrix, create_train_data::DataHierarchyType& data_sample_hierarchy);

	void loadTextureDatabaseLabeledAttributeFeatures(std::string filename, cv::Mat& ground_truth_attribute_matrix, cv::Mat& class_label_matrix, create_train_data::DataHierarchyType& data_sample_hierarchy);

	// saves the training indices, computed attributes and class labels for each fold of an attribute learning cross-validation
	void saveAttributeCrossValidationData(std::string path, const std::vector< std::vector<int> >& preselected_train_indices, const std::vector<cv::Mat>& attribute_matrix_test_data, const std::vector<cv::Mat>& class_label_matrix_test_data, const std::vector<cv::Mat>& computed_attribute_matrices);
	void loadAttributeCrossValidationData(std::string path, std::vector< std::vector<int> >& preselected_train_indices, std::vector<cv::Mat>& attribute_matrix_test_data, std::vector<cv::Mat>& class_label_matrix_test_data, std::vector<cv::Mat>& computed_attribute_matrices);

	// feature_matrix: matrix of features that should be mapped to the provided ground truth attributes
	// attribute matrix: ground truth attribute matrix
	// int cross_validation_mode: 0=leave out one object per class, 1=leave out one class
	// return_set_data: if true, all the training index sets, testing data and testing labels of all folds are returned
	// return_computed_attribute_matrices: if true, a vector of size folds that contains all the computed attributes from each fold of the cross validation is returned
	// computed_attribute_matrices: is a return vector of size folds that contains all the computed attributes from each fold of the cross validation, the order of samples is the same as in feature_matrix and attribute_matrix
	void crossValidation(unsigned int folds, const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy, CrossValidationMode cross_validation_mode);
	void crossValidation(unsigned int folds, const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy, CrossValidationMode cross_validation_mode, std::vector<cv::Mat>& computed_attribute_matrices);
	void crossValidation(unsigned int folds, const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy, CrossValidationMode cross_validation_mode,

			bool return_set_data, const cv::Mat& class_label_matrix, std::vector< std::vector<int> >& preselected_train_indices, std::vector<cv::Mat>& attribute_matrix_test_data, std::vector<cv::Mat>& class_label_matrix_test_data,
			bool return_computed_attribute_matrices, std::vector<cv::Mat>& computed_attribute_matrices);

	void displayAttributes(const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy, int display_class, bool update=false, bool store_on_disk=false);

private:
	cv::Mat attribute_display_mat_;
	int attribute_display_mat_plot_counter_;
};

#endif /* ATTRIBUTE_LEARNING_H_ */
