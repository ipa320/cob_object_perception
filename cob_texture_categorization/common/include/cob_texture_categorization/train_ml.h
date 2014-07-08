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

	void load_texture_database_features(std::string path, cv::Mat& ground_truth_attribute_matrix, cv::Mat& attribute_matrix, cv::Mat& class_label_matrix, create_train_data::DataHierarchyType& data_sample_hierarchy);
	void cross_validation(int folds, const cv::Mat& feature_matrix, const cv::Mat& label_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy,
			const std::vector< std::vector<int> >& preselected_train_indices=std::vector< std::vector<int> >(), const std::vector<cv::Mat>& feature_matrix_test_data=std::vector<cv::Mat>(), const std::vector<cv::Mat>& label_matrix_test_data=std::vector<cv::Mat>());

	void run_ml(double val, std::string *path_);

};
#endif /* TRAIN_ML_H_ */
