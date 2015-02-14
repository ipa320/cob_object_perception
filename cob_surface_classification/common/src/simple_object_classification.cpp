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
	std::cout << "=================================================\n";
	for (ST::Graph::ClusterPtr c = graph->clusters()->begin(); c != graph->clusters()->end(); ++c)
	{
		if (c->type == I_CYL)
		{
			const Eigen::Vector3f centroid = c->getCentroid();
			const Eigen::Vector3f normal = c->getOrientation();
			graph->clusters()->computeCurvature(c);
			const Eigen::Vector3f local_y_axis = normal.cross(c->min_curvature_direction);
			Eigen::Matrix3f T;
			T << c->min_curvature_direction(0), local_y_axis(0), normal(0), c->min_curvature_direction(1), local_y_axis(1), normal(1), c->min_curvature_direction(2), local_y_axis(2), normal(2);
			pcl::PointXYZ min_point(1e10, 1e10, 1e10);
			pcl::PointXYZ max_point(-1e10, -1e10, -1e10);
			for (ST::Graph::ClusterType::iterator it = c->begin(); it != c->end(); ++it)
			{
				pcl::PointXYZRGB& point = (*cloud)[*it];
				const Eigen::Vector3f tpoint = T * (Eigen::Vector3f(point.x, point.y, point.z)-centroid);
				min_point.x = std::min(min_point.x, tpoint(0));
				min_point.y = std::min(min_point.y, tpoint(1));
				min_point.z = std::min(min_point.z, tpoint(2));
				max_point.x = std::max(max_point.x, tpoint(0));
				max_point.y = std::max(max_point.y, tpoint(1));
				max_point.z = std::max(max_point.z, tpoint(2));
			}
			double bb_diag_height = max_point.x - min_point.x; //sqrt((max_point.x-min_point.x)*(max_point.x-min_point.x)+(max_point.y-min_point.y)*(max_point.y-min_point.y)+(max_point.z-min_point.z)*(max_point.z-min_point.z));
			const float radius = 0.5*(max_point.y-min_point.y);
			if (bb_diag_height > 0.12)
				std::cout << "-------------------------------------------------"
					<< "\nCentroid: (" << centroid(0) << ", " << centroid(1) << ", " << centroid(2) << ")"
					<< "\nmin_point: (" << min_point.x << ", " << min_point.y << ", " << min_point.z << ")"
					<< "\nmax_point: (" << max_point.x << ", " << max_point.y << ", " << max_point.z << ")"
					<< "\nBounding box diagonal length: " << bb_diag_height
//					<< "\nc->max_curvature: " << c->max_curvature //<< "\tradius: " << 1./c->max_curvature
					<< "\nc->min_curvature: " << c->min_curvature //<< "\tradius: " << 1./c->min_curvature
					<< "\nradius: " << radius << std::endl;
//					<< "\nc->min_curvature_direction: (" << c->min_curvature_direction(0) << ", " << c->min_curvature_direction(1) << ", " << c->min_curvature_direction(2) << ")"
//					<< "\nc->min_curvature_direction: (" << c->pca_inter_comp1(0) << ", " << c->pca_inter_comp1(1) << ", " << c->pca_inter_comp1(2) << ")" << std::endl;
			if (fabs(bb_diag_height - 0.20) < 0.025 && c->min_curvature < 0.01 && fabs(radius - 0.04) < 0.015)
			{
				// found a Pringles sized cylinder
				std::cout << "---> Pringles detection <---" << std::endl;
			}
		}
	}
}
