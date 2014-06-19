/*
 * create_train_data.h
 *
 *  Created on: 13.01.2014
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef CREATE_TRAIN_DATA_H_
#define CREATE_TRAIN_DATA_H_

#include <cob_texture_categorization/texture_categorization.h>


class create_train_data
{
public:
	typedef std::vector< std::vector< std::vector< int > > > DataHierarchyType;

	create_train_data();

	std::vector<std::string> get_texture_classes();

	void compute_data(std::string *path_data, int status, std::string *path_save, int number_pictures);

	void load_data_hierarchy(std::string filename, DataHierarchyType& data_sample_hierarchy);

private:
	std::vector<std::string> texture_classes_;
};
#endif /* CREATE_TRAIN_DATA_H_ */
