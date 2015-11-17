/*
 * train_kneighbor.h
 *
 *  Created on: 13.01.2014
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef TRAIN_ML_H_
#define TRAIN_ML_H_

//#include "cob_texture_categorization/texture_categorization.h"
#include "cob_texture_categorization/create_train_data.h"
#include "cob_texture_categorization/ml_params.h"

#include <ml.h>

#include <set>
#include <map>


class train_ml
{
public:
	train_ml();

	// do a leave out one object per class cross validation on class prediction
	void cross_validation(const CrossValidationParams& cross_validation_params, const cv::Mat& feature_matrix, const cv::Mat& label_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy,
			const std::vector< std::vector<int> >& preselected_train_indices=std::vector< std::vector<int> >(), const std::vector<cv::Mat>& feature_matrix_test_data=std::vector<cv::Mat>(),
			const std::vector<cv::Mat>& label_matrix_test_data=std::vector<cv::Mat>(), const std::vector<cv::Mat>& feature_matrices=std::vector<cv::Mat>());

	// do a leave out one class cross validation by training that left out class with generated features only and testing the performance with the respective real data of this class
	// the remaining classes are trained each cycle with their real data but not tested
	// computed_attribute_matrices: is a vector that contains the computed attributes for the whole database but for each matrix one class (the one with the vector's index) did not contribute training data to the attribute classifiers
	void cross_validation_with_generated_attributes(int folds, const std::vector<cv::Mat>& computed_attribute_matrices, const cv::Mat& class_label_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy,
			const cv::Mat& generated_attributes_matrix, const cv::Mat& generated_attributes_class_label_matrix, const create_train_data::DataHierarchyType generated_attributes_data_sample_hierarchy);

	void train(const cv::Mat& training_data, const cv::Mat& training_labels);
	void predict(const cv::Mat& test_data, const cv::Mat& test_labels, cv::Mat& predicted_labels);

	void save_mlp(std::string path);
	void load_mlp(std::string path);

	void save_computed_attribute_matrices(std::string path, const std::vector<cv::Mat>& computed_attribute_matrices);
	void load_computed_attribute_matrices(std::string path, std::vector<cv::Mat>& computed_attribute_matrices);

//	void newClassTest(const cv::Mat& feature_matrix, const cv::Mat& label_matrix,const cv::Mat& orig);
//	void run_ml(double val, std::string *path_);

private:
	CvANN_MLP mlp_;
	std::map<float, float> label_class_mapping_;	// maps the original data label (first) to a class number (second) between 0 and number_classes
	std::map<float, float> class_label_mapping_;	// maps a class number (first) between 0 and number_classes to the original data label (second)
};
#endif /* TRAIN_ML_H_ */
