/*
 * perspective_transformation.h
 *
 *  Created on: 13.01.2014
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef PCA_TEST_H_
#define PCA_TEST_H_

#include <cob_texture_categorization/texture_categorization.h>


class p_transformation
{
public:
	p_transformation();
	void run_pca(cv::Mat *source, cv::Mat *depth, const sensor_msgs::PointCloud2ConstPtr& pointcloud, visualization_msgs::MarkerArray* marker);
};
#endif /* PERSPECTIVE_TRANSFORMATION_H_ */
