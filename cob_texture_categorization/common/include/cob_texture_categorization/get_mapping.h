/*
 * get_mapping.h
 *
 *  Created on: 11.10.2013
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef GET_MAPPING_H
#define GET_MAPPING_H

#include <create_lbp.h>
#include <string>


class get_mapping
{
public:
	get_mapping();
	void get_mapping_res(int samples, std::string mappingtype,struct mapping *mapping, int *table);

};
#endif
