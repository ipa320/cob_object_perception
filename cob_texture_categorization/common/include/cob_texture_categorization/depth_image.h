/*
 * depth_image.h
 *
 *  Created on: 13.01.2014
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef DEPTH_IMAGE_H_
#define DEPTH_IMAGE_H_

#include <cob_texture_categorization/texture_categorization.h>


class depth_image
{
public:
	depth_image();
	void medianfilter(cv::Mat *depth_image);
	void close_operation(cv::Mat *depth_image);
	void get_depth_image(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg, cv::Mat *depth);
};
#endif /* DEPTH_IMAGE_H_ */
