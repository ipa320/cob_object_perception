/*
 * perspective_transformation.h
 *
 *  Created on: 13.01.2014
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef PCA_TEST_H_
#define PCA_TEST_H_

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
//#include <pcl/filters/conditional_removal.h>
#include <opencv2/opencv.hpp>
//#include <visualization_msgs/MarkerArray.h>


class PerspectiveTransformation
{
public:
	PerspectiveTransformation();

	// normalized_resolution: desired resolution of the normalized perspective in [pixel/m]
	bool normalize_perspective(cv::Mat& image, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, std::vector<float>& plane_coeff, cv::Mat& H_, const double normalized_resolution = 300., const pcl::IndicesPtr indices = pcl::IndicesPtr());

};
#endif /* PERSPECTIVE_TRANSFORMATION_H_ */
