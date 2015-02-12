/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2015 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: Care-O-bot
* \note
* ROS stack name: cob_object_perception
* \note
* ROS package name: cob_surface_classification
*
* \author
* Author: Richard Bormann
* \author
* Supervised by:
*
* \date Date of creation: 12.02.2015
*
* \brief
* simple object classification capabilities from point cloud segments
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/

#include "cob_surface_classification/simple_object_classification.h"

#include <iostream>

SimpleObjectClassification::SimpleObjectClassification()
{

}

SimpleObjectClassification::~SimpleObjectClassification()
{

}

//void SimpleObjectClassification::segmented_pointcloud_callback(const cob_surface_classification::SegmentedPointCloud2& segmented_pointcloud_msg)
//{
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::fromROSMsg(segmented_pointcloud_msg.pointcloud, *cloud);
//
//	displaySegmentedPointCloud(cloud);
//}

void SimpleObjectClassification::displaySegmentedPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, ST::Graph::Ptr graph)
{
	const int height = cloud->height;
	const int width = cloud->width;
	cv::Mat segmentation_3d = cv::Mat::zeros(height, width, CV_8UC3);
	for (ST::Graph::ClusterPtr c = graph->clusters()->begin(); c != graph->clusters()->end(); ++c)
	{
		const cv::Vec3b random_color(rand()%128, rand()%128, rand()%128);
		for (ST::Graph::ClusterType::iterator it = c->begin(); it != c->end(); ++it)
		{
			int x = *it%width;
			int y = *it/width;
			pcl::PointXYZRGB& point = (*cloud)[*it];
			cv::Vec3b image_color(point.b, point.g, point.r);
			segmentation_3d.at<cv::Vec3b>(y,x)+=2*random_color;//image_color +
		}
	}

	cv::imshow("3d segmentation", segmentation_3d);
	int key = cv::waitKey();
	if (key == 'q')
		exit(0);
}

void SimpleObjectClassification::classifyObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, ST::Graph::Ptr graph)
{
	for (ST::Graph::ClusterPtr c = graph->clusters()->begin(); c != graph->clusters()->end(); ++c)
	{
		if (c->type == I_CYL)
		{
			pcl::PointXYZ min_point(1e10, 1e10, 1e10);
			pcl::PointXYZ max_point(-1e10, -1e10, -1e10);
			for (ST::Graph::ClusterType::iterator it = c->begin(); it != c->end(); ++it)
			{
				pcl::PointXYZRGB& point = (*cloud)[*it];
				min_point.x = std::min(min_point.x, point.x);
				min_point.y = std::min(min_point.y, point.y);
				min_point.z = std::min(min_point.z, point.z);
				max_point.x = std::max(max_point.x, point.x);
				max_point.y = std::max(max_point.y, point.y);
				max_point.z = std::max(max_point.z, point.z);
			}
			graph->clusters()->computeCurvature(c);
			std::cout << "-------------------------------------------------"
					<< "\nCentroid: (" << c->getCentroid()(0) << ", " << c->getCentroid()(1) << ", " << c->getCentroid()(2) << ")"
					<< "\nBounding box diagonal length: " << sqrt((max_point.x-min_point.x)*(max_point.x-min_point.x)+(max_point.y-min_point.y)*(max_point.y-min_point.y)+(max_point.z-min_point.z)*(max_point.z-min_point.z))
					<< "\nc->max_curvature: " << c->max_curvature
					<< "\nc->min_curvature: " << c->min_curvature
					<< "\nc->min_curvature_direction: (" << c->min_curvature_direction(0) << ", " << c->min_curvature_direction(1) << ", " << c->min_curvature_direction(2) << ")" << std::endl;
		}
	}
}
