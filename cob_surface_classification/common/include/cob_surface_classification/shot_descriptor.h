#pragma once

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/features/shot.h>



class ShotHistogramDescriptor
{
public:
	void computeShotDescriptors(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input_cloud, const pcl::PointCloud<pcl::Normal>::ConstPtr& normals, const pcl::IndicesConstPtr& indices, pcl::PointCloud<pcl::SHOT352>::Ptr& shot_descriptors);
};
