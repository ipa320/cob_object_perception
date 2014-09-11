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
#include <opencv/cv.h>


class create_train_data
{
public:
	typedef std::vector< std::vector< std::vector< int > > > DataHierarchyType;   // data_sample_hierarchy[class_index][object_index][sample_index] = entry_index in feature data matrix

	create_train_data();

	std::vector<std::string> get_texture_classes();

	void compute_data_handcrafted(std::string path_database_images, std::string path_save, int number_pictures, int mode=0);
	void compute_data_cimpoi(std::string path_database_images, std::string path_save, int number_pictures, int mode=0, bool generateGMM=false);

	void save_texture_database_features(std::string path, const cv::Mat& base_feature_matrix, const cv::Mat& ground_truth_attribute_matrix, const cv::Mat& computed_attribute_matrix, const cv::Mat& class_label_matrix, DataHierarchyType& data_sample_hierarchy, int mode=0);
	void load_texture_database_features(std::string path, cv::Mat& base_feature_matrix, cv::Mat& ground_truth_attribute_matrix, cv::Mat& computed_attribute_matrix, cv::Mat& class_label_matrix, DataHierarchyType& data_sample_hierarchy);

	void save_data_hierarchy(std::string filename, DataHierarchyType& data_sample_hierarchy, int number_samples);
	void load_data_hierarchy(std::string filename, DataHierarchyType& data_sample_hierarchy);

	void load_filenames_gt_attributes(std::string filename, std::map<std::string, std::vector<float> >& filenames_gt_attributes);

private:
	std::vector<std::string> texture_classes_;
};
#endif /* CREATE_TRAIN_DATA_H_ */
