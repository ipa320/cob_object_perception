#ifndef XML_WRITE_H
#define XML_WRITE_H

#include <cob_texture_categorization/texture_features.h>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

struct data{
	std::string data_in[6];
	int param[20];
};
struct stat{
	int create;
	std::string upper;
};


class xml_write
{
public:
	xml_write();
	void getstat(std::string path, struct data *daten, struct stat *status);
	std::string write_dataset(struct data*, int, struct feature_results *results);
	void write_data(struct stat *status, std::string path, struct data *daten, struct feature_results *results);
	void write_main(struct feature_results *results, struct data *daten, struct stat *status);
};


#endif
