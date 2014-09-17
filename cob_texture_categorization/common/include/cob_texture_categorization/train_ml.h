/*
 * train_kneighbor.h
 *
 *  Created on: 13.01.2014
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef TRAIN_ML_H_
#define TRAIN_ML_H_

#include "cob_texture_categorization/texture_categorization.h"
#include "cob_texture_categorization/create_train_data.h"


class train_ml
{
public:
	train_ml();

	// do a leave out one object per class cross validation on class prediction
	void cross_validation(int folds, const cv::Mat& feature_matrix, const cv::Mat& label_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy,
			const std::vector< std::vector<int> >& preselected_train_indices=std::vector< std::vector<int> >(), const std::vector<cv::Mat>& feature_matrix_test_data=std::vector<cv::Mat>(), const std::vector<cv::Mat>& label_matrix_test_data=std::vector<cv::Mat>());

	// do a leave out one class cross validation by training that left out class with generated features only and testing the performance with the respective real data of this class
	// the remaining classes are trained each cycle with their real data but not tested
	// computed_attribute_matrices: is a vector that contains the computed attributes for the whole database but for each matrix one class (the one with the vector's index) did not contribute training data to the attribute classifiers
	void cross_validation_with_generated_attributes(int folds, const std::vector<cv::Mat>& computed_attribute_matrices, const cv::Mat& class_label_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy,
			const cv::Mat& generated_attributes_matrix, const cv::Mat& generated_attributes_class_label_matrix, const create_train_data::DataHierarchyType generated_attributes_data_sample_hierarchy);


	void newClassTest(const cv::Mat& feature_matrix, const cv::Mat& label_matrix,const cv::Mat& orig);
	void run_ml(double val, std::string *path_);

};
#endif /* TRAIN_ML_H_ */
