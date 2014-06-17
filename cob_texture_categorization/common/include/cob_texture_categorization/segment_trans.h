/*
 * segment_trans.h
 *
 *  Created on: 13.01.2014
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef SEGMENT_TRANS_H_
#define SEGMENT_TRANS_H_

#include <cob_texture_categorization/texture_categorization.h>


class segment_trans
{
public:
	segment_trans();
	void transformation(cv::Mat *source, cv::Mat *dest, cv::Mat *depth);
};
#endif /* SEGMENT_TRANS_H_ */
