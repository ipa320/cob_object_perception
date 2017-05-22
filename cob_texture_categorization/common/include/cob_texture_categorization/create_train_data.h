/*
 * create_train_data.h
 *
 *  Created on: 13.01.2014
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef CREATE_TRAIN_DATA_H_
#define CREATE_TRAIN_DATA_H_

#include <vector>
#include <string>

// opencv
#include <opencv2/opencv.hpp>

#include "cob_texture_categorization/ifv_features.h"

class create_train_data
{
public:
	typedef std::vector< std::vector< std::vector< int > > > DataHierarchyType;   // data_sample_hierarchy[class_index][object_index][sample_index] = entry_index in feature data matrix

	create_train_data(int database=0);

	std::vector<std::string> get_texture_classes();

	// computes feature and label matrices of the provided database
	// @param database_identifier defines the database to load (i.e. "ipa", "dtd"), necessary for dealing with the different file formats
	void compute_data_handcrafted(std::string path_database_images, std::string path_save, const std::string& database_identifier);
	void compute_data_cimpoi(std::string path_database_images, std::string path_save, const std::string& database_identifier, bool generateGMM=false, IfvFeatures::FeatureType feature_type=IfvFeatures::DENSE_MULTISCALE_SIFT);

	void save_texture_database_features(const std::string path, const cv::Mat& base_feature_matrix, const cv::Mat& ground_truth_attribute_matrix, const cv::Mat& computed_attribute_matrix, const cv::Mat& class_label_matrix, DataHierarchyType& data_sample_hierarchy, const std::vector<std::string>& image_filenames, const std::string& database_identifier);
	void load_texture_database_features(std::string path, cv::Mat& base_feature_matrix, cv::Mat& ground_truth_attribute_matrix, cv::Mat& computed_attribute_matrix, cv::Mat& class_label_matrix, DataHierarchyType& data_sample_hierarchy, std::vector<std::string>& image_filenames, const std::string& database_identifier);

	void save_data_hierarchy(const std::string filename, DataHierarchyType& data_sample_hierarchy, const std::vector<std::string>& image_filenames, const int number_samples);
	void load_data_hierarchy(std::string filename, DataHierarchyType& data_sample_hierarchy, std::vector<std::string>& image_filenames);

	void load_filenames_gt_attributes(std::string filename, std::vector<std::string>& class_names, std::vector<std::string>& image_filenames, std::map<std::string, std::vector<float> >& filenames_gt_attributes, DataHierarchyType& data_sample_hierarchy, const std::string& database_identifier);

	// initial setup of the dtd_database.txt file
	void create_dtd_database_file(std::string path_database_images, std::string path_save, std::string label_file);

private:
	std::vector<std::string> texture_classes_;
};
#endif /* CREATE_TRAIN_DATA_H_ */
