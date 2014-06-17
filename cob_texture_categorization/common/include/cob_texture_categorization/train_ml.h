/*
 * train_kneighbor.h
 *
 *  Created on: 13.01.2014
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef TRAIN_ML_H_
#define TRAIN_ML_H_

#include <cob_texture_categorization/texture_categorization.h>


class train_ml
{
public:
	train_ml();
	void run_ml(double val, std::string *path_);

};
#endif /* TRAIN_ML_H_ */
