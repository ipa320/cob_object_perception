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

	void load_texture_database_features(std::string path, cv::Mat& feature_matrix, cv::Mat& label_matrix, create_train_data::DataHierarchyType& data_sample_hierarchy);
	void cross_validation(int folds, const cv::Mat& feature_matrix, const cv::Mat& label_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy);

	void run_ml(double val, std::string *path_);

};
#endif /* TRAIN_ML_H_ */
