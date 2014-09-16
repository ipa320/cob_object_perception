/*
 * perspective_transformation.h
 *
 *  Created on: 13.01.2014
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef PCA_TEST_H_
#define PCA_TEST_H_

#include <cob_texture_categorization/texture_categorization.h>

//#include <iostream>
#include <pcl/point_types.h>
//#include <pcl/filters/conditional_removal.h>

class p_transformation
{
public:
	p_transformation();
	bool run_pca(cv::Mat *source, cv::Mat *depth, pcl::PointCloud< pcl::PointXYZ >::Ptr pixelpointcloud, pcl::PointCloud< pcl::PointXYZ >::Ptr metricpointcloud, visualization_msgs::MarkerArray* marker, std::vector<float>* plane_coeff, cv::Mat *H_);
//	const sensor_msgs::PointCloud2ConstPtr&
//	pcl::PointCloud< pcl::PointXYZ >::Ptr
};
#endif /* PERSPECTIVE_TRANSFORMATION_H_ */
