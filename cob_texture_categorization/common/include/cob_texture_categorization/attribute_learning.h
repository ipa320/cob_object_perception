/*
 * attribute_learning.h
 *
 *  Created on: 28.06.2014
 *      Author: Richard Bormann
 */

#ifndef ATTRIBUTE_LEARNING_H_
#define ATTRIBUTE_LEARNING_H_

#include "cob_texture_categorization/create_train_data.h"
#include "cob_texture_categorization/ml_params.h"

#include <opencv2/opencv.hpp>
#include <vector>
#include <boost/shared_ptr.hpp>


class AttributeLearning
{
public:
	//enum CrossValidationMode {LEAVE_OUT_ONE_OBJECT_PER_CLASS = 0, LEAVE_OUT_ONE_CLASS = 1};

	AttributeLearning() {};

	void loadTextureDatabaseBaseFeatures(const std::string& filename, const int feature_number, cv::Mat& feature_matrix, cv::Mat& ground_truth_attribute_matrix, cv::Mat& class_label_matrix, create_train_data::DataHierarchyType& data_sample_hierarchy, std::vector<std::string>& image_filenames, const std::string& database_identifier);

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
	void crossValidation(const CrossValidationParams& cross_validation_params, const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy);
	void crossValidation(const CrossValidationParams& cross_validation_params, const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy,
			std::vector<cv::Mat>& computed_attribute_matrices);
	void crossValidation(const CrossValidationParams& cross_validation_params, const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy,
			bool return_set_data, const cv::Mat& class_label_matrix, std::vector< std::vector<int> >& preselected_train_indices, std::vector<cv::Mat>& attribute_matrix_test_data, std::vector<cv::Mat>& class_label_matrix_test_data,
			bool return_computed_attribute_matrices, std::vector<cv::Mat>& computed_attribute_matrices);

	void train(const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix);
	void predict(const cv::Mat& feature_data, cv::Mat& predicted_labels);

	void save_SVMs(std::string path);
	void load_SVMs(std::string path);

	void displayAttributes(const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy, int display_class, bool update=false, bool store_on_disk=false);

	// DTD specific

	size_t sampleIndexFromFilename(const std::string& filename, const std::vector<std::string>& indexed_filenames);
	void loadDTDDatabaseCrossValidationSet(const std::string& path_to_cross_validation_sets, const std::string& set_type, const std::vector<std::string>& image_filenames, const int fold,
					const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, cv::Mat& feature_matrix_set, cv::Mat& attribute_matrix_set);
	void loadDTDDatabaseCrossValidationSets(const std::string& path_to_cross_validation_sets, const std::vector<std::string>& image_filenames, const int fold,
					const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, cv::Mat& feature_matrix_train, cv::Mat& attribute_matrix_train,
					cv::Mat& feature_matrix_validation, cv::Mat& attribute_matrix_validation, cv::Mat& feature_matrix_test, cv::Mat& attribute_matrix_test);
	double computeAveragePrecisionPascal11(const std::vector<double>& recall, const std::vector<double>& precision);
	double computeAveragePrecision(const std::vector<float>& ground_truth_labels, const std::vector<float>& prediction_scores, float& max_f_score);
	void crossValidationDTD(CrossValidationParams& cross_validation_params, const std::string& path_to_cross_validation_sets, const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy, const std::vector<std::string>& image_filenames);

private:
	cv::Mat attribute_display_mat_;
	int attribute_display_mat_plot_counter_;

#if CV_MAJOR_VERSION == 2
	std::vector<boost::shared_ptr<cv::SVM> > svm_;
#else
	std::vector<cv::Ptr<cv::ml::SVM> > svm_;
#endif
};

#endif /* ATTRIBUTE_LEARNING_H_ */
