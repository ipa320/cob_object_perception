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
	create_train_data();
	void compute_data(std::string *path_data, int status, std::string *path_save, int number_pictures);

	void load_data_hierarchy(std::string filename, std::vector< std::vector< std::vector< int > > >& data_sample_hierarchy);

};
#endif /* CREATE_TRAIN_DATA_H_ */
