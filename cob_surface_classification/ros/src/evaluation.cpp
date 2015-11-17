/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2013 \n
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
 * \date Date of creation: 07.08.2012
 *
 * \brief
 * functions for display of people detections
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


/*
 * Evaluation.cpp
 *
 *  Created on: Jul 10, 2013
 *      Author: rmb-ce
 */

#include "cob_surface_classification/evaluation.h"

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types_conversion.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>

#include <opencv/highgui.h>


#define HUE_GREEN 		113/2
#define HUE_MAGENTA 	0/2
#define HUE_YELLOW 		55/2
#define HUE_DIFF_TH		20		//threshold for deviation in hue value which is still ignored (minimum distance between the different hue values that are used)




Evaluation::Evaluation()
{
	search_directory = std::string(getenv("HOME")) + "/scene_recordings/";

	//colors of the classification categories in the ground truth
	color_table_.resize(NUMBER_SURFACE_CLASS_TYPES);
	color_table_[I_EDGE] = LBL_EDGE; //EVAL_COL_EDGE;
	color_table_[I_PLANE] = LBL_PLANE; //EVAL_COL_PLANE;
	color_table_[I_CONCAVE]= LBL_CYL; //EVAL_COL_CONC;
	color_table_[I_CONVEX]= LBL_CYL_CVX; //EVAL_COL_CONV;

	color_table_sim_.resize(NUMBER_SURFACE_CLASS_TYPES, cv::Vec3b(64,64,64));
	color_table_sim_[I_UNDEF] = cv::Vec3b(128,128,128);
	color_table_sim_[I_EDGE] = cv::Vec3b(0,0,0);
	color_table_sim_[I_PLANE] = cv::Vec3b(51,201,69);
	color_table_sim_[I_CONCAVE] = cv::Vec3b(2,234,255);
	color_table_sim_[I_CONVEX] = cv::Vec3b(23,23,236);
}

Evaluation::~Evaluation()
{
}

void Evaluation::evaluateSurfaceTypeRecognition(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, const cv::Mat& gt_color_image)
{
	// 1. Computation of the ground truth image (= color normalization of the scene image + depth edges)
	cv::Mat gt_color_image_normalized;
	generateGroundTruthImage(gt_point_cloud, gt_color_image, gt_color_image_normalized);

	// 2. Compute color coded surface type estimate
	cv::Mat surface_estimate_image;
	generateSurfaceTypeEstimateImage(surface_estimate_image, gt_color_image_normalized.rows, gt_color_image_normalized.cols);

	// 3. Count the number of fits in the data
	const int search_radius = 1;
	std::vector<double> recall, precision;
	computePerformanceMeasures(gt_color_image_normalized, surface_estimate_image, search_radius, recall, precision);

	// 4. Displays
	std::cout << "Results on surface type estimation:\n";
	for (int i=0; i<recall.size(); ++i)
		std::cout << i << ":\trecall=" << recall[i] << "\tprecision=" << precision[i] << "\n";
	std::cout << std::endl;
//	cv::imshow("gt image", gt_color_image_normalized);
//	cv::imshow("surface estimate", surface_estimate_image);
//	int key = cv::waitKey();
//	if (key == 'q')
//		exit(0);
}

void Evaluation::evaluateEdgeRecognition(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, const cv::Mat& gt_color_image, const cv::Mat& edge_image, EdgeDetectionStatistics* edge_detection_statistics)
{
	// 1. Computation of the ground truth image (= color normalization of the scene image + depth edges)
	cv::Mat gt_color_image_normalized;
	generateGroundTruthImage(gt_point_cloud, gt_color_image, gt_color_image_normalized);

	// 2. Compute color coded edge estimate
	cv::Mat edge_estimate_image(edge_image.rows, edge_image.cols, CV_8UC3);
	edge_estimate_image.setTo(color_table_sim_[I_UNDEF]);
	for (int v=0; v<edge_image.rows; ++v)
		for (int u=0; u<edge_image.cols; ++u)
			if (edge_image.at<uchar>(v,u)!=0)
				edge_estimate_image.at<cv::Vec3b>(v,u) = color_table_sim_[I_EDGE];

	// 3. Count the number of fits in the data
	const int search_radius = 1;
	std::vector<double> recall, precision;
	computePerformanceMeasures(gt_color_image_normalized, edge_estimate_image, search_radius, recall, precision);

	// 4. Displays
	if (edge_detection_statistics != 0)
		edge_detection_statistics->addStatistics(recall[I_EDGE], precision[I_EDGE]);
//	std::cout << "Results on edge estimation:\n\trecall=" << recall[I_EDGE] << "\tprecision=" << precision[I_EDGE] << "\n\n";
//	cv::imshow("gt image", gt_color_image_normalized);
//	cv::imshow("edge estimate", edge_estimate_image);
//	int key = cv::waitKey();
//	if (key == 'q')
//		exit(0);
}

void Evaluation::generateGroundTruthImage(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, const cv::Mat& gt_color_image, cv::Mat& gt_color_image_normalized)
{
	// computation of the ground truth image (= color normalization of the scene image + depth edges)
	gt_color_image_normalized.create(gt_color_image.rows, gt_color_image.cols, CV_8UC3);
	cv::Mat gt_color_image_hsv;
	cv::cvtColor(gt_color_image, gt_color_image_hsv, CV_BGR2HSV);

	for (int v=0; v<gt_color_image_hsv.rows; ++v)
	{
		for (int u=0; u<gt_color_image_hsv.cols; ++u)
		{
			// check for depth edges
			bool depth_edge = checkDepthEdge(gt_point_cloud, u, v);

			// determine color coding class from gt image
			cv::Vec3b& color_hsv = gt_color_image_hsv.at<cv::Vec3b>(v,u);
			if (depth_edge)	// || color_hsv.val[2] < 0.38*255)
				gt_color_image_normalized.at<cv::Vec3b>(v,u) = color_table_sim_[I_EDGE];
			else if (abs(color_hsv.val[0]-HUE_GREEN) < HUE_DIFF_TH)
				gt_color_image_normalized.at<cv::Vec3b>(v,u) =  color_table_sim_[I_PLANE];
			else if (abs(color_hsv.val[0]-HUE_MAGENTA) < HUE_DIFF_TH)
				gt_color_image_normalized.at<cv::Vec3b>(v,u) =  color_table_sim_[I_CONVEX];
			else if (abs(color_hsv.val[0]-HUE_YELLOW) < HUE_DIFF_TH)
				gt_color_image_normalized.at<cv::Vec3b>(v,u) =  color_table_sim_[I_CONCAVE];
		}
	}
}

void Evaluation::generateSurfaceTypeEstimateImage(cv::Mat& color_image, const int height, const int width)
{
	color_image.create(height, width, CV_8UC3);
	color_image.setTo(color_table_sim_[I_UNDEF]);

	ST::CH::ClusterPtr c_it,c_end;

	for ( boost::tie(c_it,c_end) = clusterHandler->getClusters(); c_it != c_end; ++c_it)
	{
		const cv::Vec3b color = color_table_sim_[c_it->type];
		for (ST::CH::ClusterType::iterator idx=c_it->begin(); idx != c_it->end(); ++idx)
		{
			int v = *idx/width;	//row in image
			int u = *idx%width;	//column
			color_image.at<cv::Vec3b>(v,u) = color;
		}
	}
}

bool Evaluation::checkDepthEdge(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, int u, int v)
{
	if (u!=0 && v!=0 && u!=gt_point_cloud->width-1 && v!=gt_point_cloud->height-1)
	{
		// check for depth edges
		const double depth_factor = 0.01;
		const double depth = gt_point_cloud->at(u,v).z;
		const double edge_threshold = depth_factor*depth;
		const double dzl = gt_point_cloud->at(u,v).z - gt_point_cloud->at(u-1,v).z;
		const double dzr = gt_point_cloud->at(u+1,v).z - gt_point_cloud->at(u,v).z;
		const double dzu = gt_point_cloud->at(u,v).z - gt_point_cloud->at(u,v-1).z;
		const double dzb = gt_point_cloud->at(u,v+1).z - gt_point_cloud->at(u,v).z;
		if ( ((dzr<-edge_threshold || dzr>edge_threshold) && fabs(dzl-dzr)>0.01) || ((dzb<-edge_threshold || dzb>edge_threshold) && fabs(dzu-dzb)>0.01) )
			return true;
		// additonally check for surface edges
		const double min_detectable_edge_angle = 35.;
		const double alpha_left = atan2(-dzl, -(gt_point_cloud->at(u,v).x - gt_point_cloud->at(u-1,v).x));
		const double alpha_right = atan2(dzr, gt_point_cloud->at(u+1,v).x - gt_point_cloud->at(u,v).x);
		double diff = fabs(alpha_left - alpha_right);
		if (diff!=0 && (diff < (180.-min_detectable_edge_angle) / 180. * CV_PI || diff > (180.+min_detectable_edge_angle) / 180. * CV_PI))
			return true;
		const double alpha_upper = atan2(-dzu, -(gt_point_cloud->at(u,v).y - gt_point_cloud->at(u,v-1).y));
		const double alpha_below = atan2(dzb, gt_point_cloud->at(u,v+1).y - gt_point_cloud->at(u,v).y);
		diff = fabs(alpha_upper - alpha_below);
		if (diff!=0 && (diff < (180.-min_detectable_edge_angle) / 180. * CV_PI || diff > (180.+min_detectable_edge_angle) / 180. * CV_PI))
			return true;
	}
	return false;
}

void Evaluation::computePerformanceMeasures(const cv::Mat& gt_image, const cv::Mat& estimate, const int search_radius, std::vector<double>& recall, std::vector<double>& precision)
{
	const int padding = 20; 	// todo: link this parameter to max_line_width from edge detection
	// 1. recall
	recall.clear();
	recall.resize(NUMBER_SURFACE_CLASS_TYPES, 0.);
	std::vector<int> surface_type_counter(NUMBER_SURFACE_CLASS_TYPES, 0);
	for (int v=padding; v<gt_image.rows-padding; ++v)
	{
		for (int u=padding; u<gt_image.cols-padding; ++u)
		{
			bool correct = false;
			const int gt_label = getSurfaceTypeFromColor(gt_image.at<cv::Vec3b>(v,u));
			surface_type_counter[gt_label]++;
			for (int dv=-search_radius; dv<=search_radius && correct==false; ++dv)
			{
				for (int du=-search_radius; du<=search_radius && correct==false; ++du)
				{
					const int x = u+du;
					const int y = v+dv;
					if (x<0 || x>=gt_image.cols || y<0 || y>=gt_image.rows)
						continue;
					if (getSurfaceTypeFromColor(estimate.at<cv::Vec3b>(y,x)) == gt_label)
					{
						recall[gt_label]+=1.;
						correct = true;
					}
				}
			}
		}
	}
	for (int i=0; i<recall.size(); ++i)
		recall[i] = divide(recall[i], surface_type_counter[i]);

	// 2. precision
	precision.clear();
	precision.resize(NUMBER_SURFACE_CLASS_TYPES, 0.);
	surface_type_counter.clear();
	surface_type_counter.resize(NUMBER_SURFACE_CLASS_TYPES, 0);
	for (int v=padding; v<estimate.rows-padding; ++v)
	{
		for (int u=padding; u<estimate.cols-padding; ++u)
		{
			bool correct = false;
			const int estimate_label = getSurfaceTypeFromColor(estimate.at<cv::Vec3b>(v,u));
			surface_type_counter[estimate_label]++;
			for (int dv=-search_radius; dv<=search_radius && correct==false; ++dv)
			{
				for (int du=-search_radius; du<=search_radius && correct==false; ++du)
				{
					const int x = u+du;
					const int y = v+dv;
					if (x<0 || x>=estimate.cols || y<0 || y>=estimate.rows)
						continue;
					if (getSurfaceTypeFromColor(gt_image.at<cv::Vec3b>(y,x)) == estimate_label)
					{
						precision[estimate_label]+=1.;
						correct = true;
					}
				}
			}
		}
	}
	for (int i=0; i<precision.size(); ++i)
		precision[i] = divide(precision[i], surface_type_counter[i]);
}

int Evaluation::getSurfaceTypeFromColor(const cv::Vec3b& color)
{
	for (int i=0; i<color_table_sim_.size(); ++i)
	{
		if (color_table_sim_[i] == color)
			return i;
	}

	return I_UNDEF;
}

/*
int Evaluation::compareClassification(std::string gt_filename)
{


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr gt (new pcl::PointCloud<pcl::PointXYZRGB>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (search_directory + gt_filename, *gt) == -1)
	{
		PCL_ERROR ("Couldn't read gt-file \n");
		return (-1);
	}

	compareClassification(gt,...colorimage...);
}

 */

double Evaluation::divide(double a, double b)
{
	//compute a/b
	if (b == 0)
	{
		if(a == 0)
			return 100;
		else
			return 0;
	}
	else
		return (a / b) * 100;
}


void Evaluation::evaluateNormalEstimation(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals, NormalEstimationStatistics* ne_statistics)
{
	// 1. Estimate ground truth normals from gt_point_cloud
	pcl::PointCloud<pcl::Normal>::Ptr gt_normals(new pcl::PointCloud<pcl::Normal>);
	computeGroundTruthNormals(gt_point_cloud, gt_normals);

	// 2. Compute error of estimated normals
	const int padding = 20; 	// todo: link this parameter to max_line_width from edge detection
	int number_gt_normals = 0, number_normals = 0, number_good_normals = 0;
	double normal_error = 0., normal_error_deg = 0.;
	computeNormalEstimationError(gt_point_cloud, gt_normals, normals, padding, number_gt_normals, number_normals, number_good_normals, normal_error, normal_error_deg);

	// 3. Visualize
	if (ne_statistics != 0)
		ne_statistics->addStatistics(100.*(double)number_normals/(double)number_gt_normals, normal_error/(double)number_normals, normal_error_deg/(double)number_normals, 100.*(double)number_good_normals/(double)number_normals);
//	std::cout << "Coverage of estimated normals on gt_normals: " << 100.*(double)number_normals/(double)number_gt_normals << std::endl;
//	std::cout << "Average normal estimation error: " << normal_error/(double)number_normals << std::endl;
//	std::cout << "Average normal estimation error [deg]: " << normal_error_deg/(double)number_normals << std::endl;
//	std::cout << "Percentage of good normals: " << 100.*(double)number_good_normals/(double)number_normals << "\n" << std::endl;

//	pcl::visualization::PCLVisualizer viewerNormals("Cloud and Normals");
//	viewerNormals.setBackgroundColor(0.0, 0.0, 0);
//	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbNormals(gt_point_cloud);
//	viewerNormals.addPointCloud<pcl::PointXYZRGB>(gt_point_cloud, rgbNormals, "gt_point_cloud");
//	//viewerNormals.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(gt_point_cloud, gt_normals, 2, 0.005, "gt_normals");
//	viewerNormals.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(gt_point_cloud, normals, 2, 0.005, "normals");
//	viewerNormals.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "gt_point_cloud");
//	while (!viewerNormals.wasStopped ())
//	{
//		viewerNormals.spinOnce();
//	}
//	viewerNormals.removePointCloud("gt_point_cloud");
}

void Evaluation::computeGroundTruthNormals(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, pcl::PointCloud<pcl::Normal>::Ptr gt_normals)
{
	gt_normals->resize(gt_point_cloud->height*gt_point_cloud->width);
	gt_normals->header = gt_point_cloud->header;
	gt_normals->height = gt_point_cloud->height;
	gt_normals->width = gt_point_cloud->width;
	gt_normals->is_dense = true;
	for (int v=1; v<gt_point_cloud->height-1; ++v)
	{
		for (int u=1; u<gt_point_cloud->width-1; ++u)
		{
			if (checkDepthEdge(gt_point_cloud, u, v)==false) // &&  && checkDepthEdge(gt_point_cloud, u, v+1)==false)
			{
				Eigen::Vector3f p, p1, p2;
				p = gt_point_cloud->at(u,v).getVector3fMap();

				bool valid_neighborhood_points = true;
				if (checkDepthEdge(gt_point_cloud, u+1, v)==false)
					p1 = gt_point_cloud->at(u+1,v).getVector3fMap();
				else if (checkDepthEdge(gt_point_cloud, u-1, v)==false)
					p1 = gt_point_cloud->at(u-1,v).getVector3fMap();
				else
					valid_neighborhood_points = false;

				if (checkDepthEdge(gt_point_cloud, u, v+1)==false)
					p2 = gt_point_cloud->at(u,v+1).getVector3fMap();
				else if (checkDepthEdge(gt_point_cloud, u, v-1)==false)
					p2 = gt_point_cloud->at(u,v-1).getVector3fMap();
				else
					valid_neighborhood_points = false;

				if (valid_neighborhood_points == true)
				{
					Eigen::Vector3f n = (p1-p).cross(p2-p);
					n.normalize();
					pcl::flipNormalTowardsViewpoint<pcl::PointXYZRGB>(gt_point_cloud->at(u,v), gt_point_cloud->sensor_origin_(0), gt_point_cloud->sensor_origin_(1), gt_point_cloud->sensor_origin_(2), n(0), n(1), n(2));
					gt_normals->at(u,v).normal_x = n(0);
					gt_normals->at(u,v).normal_y = n(1);
					gt_normals->at(u,v).normal_z = n(2);
				}
				else
				{
					gt_normals->at(u,v).normal_x = std::numeric_limits<float>::quiet_NaN();
					gt_normals->at(u,v).normal_y = std::numeric_limits<float>::quiet_NaN();
					gt_normals->at(u,v).normal_z = std::numeric_limits<float>::quiet_NaN();
				}
			}
		}
	}
}

void Evaluation::computeNormalEstimationError(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, const pcl::PointCloud<pcl::Normal>::Ptr& gt_normals,
		const pcl::PointCloud<pcl::Normal>::Ptr& normals, const int padding, int& number_gt_normals, int& number_normals, int& number_good_normals, double& normal_error, double& normal_error_deg)
{
	number_gt_normals = 0;
	number_normals = 0;
	number_good_normals = 0;
	normal_error = 0.;
	normal_error_deg = 0.;
	for (int v=padding; v<gt_point_cloud->height-padding; ++v)
	{
		for (int u=padding; u<gt_point_cloud->width-padding; ++u)
		{
			const Eigen::Vector3f n = normals->at(u,v).getNormalVector3fMap();
			const Eigen::Vector3f gt_n = gt_normals->at(u,v).getNormalVector3fMap();
			if (pcl_isnan(gt_normals->at(u,v).normal_z)==false)
			{
				// gt_normal exists
				++number_gt_normals;
				if (pcl_isnan(normals->at(u,v).normal_z)==false)
				{
					// normal estimation has also found a normal
					++number_normals;
					double d = std::max(-1., std::min(1.,(double)gt_n.dot(n)));
					normal_error += fabs(1 - d);
					normal_error_deg += 180./CV_PI*acos(d);
					if (fabs(d) > 0.97)
						++number_good_normals;
				}
			}
		}
	}
}














///// old stuff

void Evaluation::clusterTypesToColorImage(cv::Mat& test_image, unsigned int height,unsigned int width)
{
	ST::CH::ClusterPtr c_it,c_end;

	for ( boost::tie(c_it,c_end) = clusterHandler->getClusters(); c_it != c_end; ++c_it)
	{

		if(c_it->type == I_EDGE || c_it->type == I_PLANE || c_it->type == I_CONCAVE || c_it->type == I_CONVEX )
		{

			uint32_t rgb = color_table_[c_it->type];

			for (ST::CH::ClusterType::iterator idx=c_it->begin(); idx != c_it->end(); ++idx)
			{
				int v = *idx/width;	//row in image
				int u = *idx%width;	//column
				unsigned char r = rgb >> 16;
				unsigned char g = rgb >> 8 & 0xFF;
				unsigned char b = rgb & 0xFF;
				test_image.at<cv::Vec3b>(v,u)[0] = b;
				test_image.at<cv::Vec3b>(v,u)[1] = g;
				test_image.at<cv::Vec3b>(v,u)[2] = r;
			}
		}
	}
}


void Evaluation::compareImagesUsingColor(cv::Mat imOrigin, cv::Mat imComp,  Evaluation::Statistics& c)
{
	//check color of all pixels in imOrigin: determine how many of them have been colored as in imComp
	//-----------------------------------------------------------------------------------------------------


	//dont't consider border of image because of inaccurate computations due to cut neighbourhood.
	for (unsigned int v=10; v<imOrigin.rows-10; v++)
	{
		for (unsigned int u=10; u<imOrigin.cols-10; u++)
		{

			c.countCompared++;
			pcl::PointXYZHSV hsv_origin;
			pcl::PointXYZHSV hsv_comp;
			pcl::PointXYZRGB rgb_origin;
			pcl::PointXYZRGB rgb_comp;
			rgb_origin.b = imOrigin.at<cv::Vec3b>(v,u)[0];
			rgb_origin.g = imOrigin.at<cv::Vec3b>(v,u)[1];
			rgb_origin.r = imOrigin.at<cv::Vec3b>(v,u)[2];
			rgb_comp.b = imComp.at<cv::Vec3b>(v,u)[0];
			rgb_comp.g = imComp.at<cv::Vec3b>(v,u)[1];
			rgb_comp.r = imComp.at<cv::Vec3b>(v,u)[2];

			pcl::PointXYZRGBtoXYZHSV (rgb_comp, hsv_comp);
			pcl::PointXYZRGBtoXYZHSV (rgb_origin, hsv_origin);

			//black (no determining hue value)
			if(hsv_origin.v < 20)
			{
				c.countEdge++;
			}
			//other colors
			//hsv_origin.h = -1 if rgb_origin = {0,0,0} black
			else
			{
				if(std::abs((int)(hsv_origin.h - HUE_GREEN)) < HUE_DIFF_TH)			c.countPlane++;
				else if (std::abs((int)(hsv_origin.h - HUE_YELLOW)) < HUE_DIFF_TH) 	c.countConc++;
				else if (std::abs((int)(hsv_origin.h - HUE_MAGENTA)) < HUE_DIFF_TH)	c.countConv++;
			}

			//comparisons
			//both black
			if((hsv_origin.v < 20) && (hsv_comp.v < 20))
			{
				c.countCorrect++;
				c.countCorrectEdge++;
			}
			//other colors
			//hsv_origin.h = -1 if rgb_origin = {0,0,0} black
			else if((hsv_origin.h != -1) && (hsv_comp.h != -1)  && (std::abs((int)(hsv_comp.h - hsv_origin.h)) < HUE_DIFF_TH))
			{
				c.countCorrect++;
				if(std::abs((int)(hsv_origin.h - HUE_GREEN)) < HUE_DIFF_TH)			c.countCorrectPlane++;
				else if (std::abs((int)(hsv_origin.h - HUE_YELLOW)) < HUE_DIFF_TH) 	c.countCorrectConc++;
				else if (std::abs((int)(hsv_origin.h - HUE_MAGENTA)) < HUE_DIFF_TH)	c.countCorrectConv++;
			}
		}
	}
}

int Evaluation::compareClassification(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr gt, cv::Mat gt_color_image)
{
	//compare all points of type edge,plane,concave,convex to the colors of the ground truth (which has to be colored according to class label, see label_defines.h)
	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------


	struct Statistics c_r, c_p;

	cv::Mat test_image = cv::Mat::ones(gt->height, gt->width, CV_8UC3);
/*	//points not taken into account until the end remain white (->no class assigned)
	clusterTypesToColorImage(test_image, gt->height, gt->width);

	rec.saveImage(test_image,"prediction");
	rec.saveImage(gt_color_image,"gt");



	//comparison using the clusters
	//------------------------------------

	int countCorrect = 0;
	int countCorrectEdge = 0;
	int countCorrectPlane = 0;
	int countCorrectConc = 0;
	int countCorrectConv = 0;
	int countCompared = 0;
	int countNoColorAssigned = 0;
	int countEdge = 0;
	int countPlane = 0;
	int countConc = 0;
	int countConv = 0;


	ST::CH::ClusterPtr c_it,c_end;
	for ( boost::tie(c_it,c_end) = clusterHandler->getClusters(); c_it != c_end; ++c_it)
	{

		if(c_it->type == I_EDGE || c_it->type == I_PLANE || c_it->type == I_CONCAVE || c_it->type == I_CONVEX )
		{
			uint32_t rgb = color_tab[c_it->type];

			for (ST::CH::ClusterType::iterator idx=c_it->begin(); idx != c_it->end(); ++idx)
			{
				countCompared++;
				int v = *idx/gt->width;	//row in image
				int u = *idx%gt->width;	//column
				//test_image.at<unsigned char>(v,u) = cvScalarAll(rgb);
				unsigned char r = rgb >> 16;
				unsigned char g = rgb >> 8 & 0xFF;
				unsigned char b = rgb & 0xFF;
				test_image.at<cv::Vec3b>(v,u)[0] = b;
				test_image.at<cv::Vec3b>(v,u)[1] = g;
				test_image.at<cv::Vec3b>(v,u)[2] = r;

				switch(c_it->type)
				{
				case I_EDGE: countEdge++; break;
				case I_PLANE: countPlane++; break;
				case I_CONCAVE: countConc++; break;
				case I_CONVEX: countConv++; break;
				}

				pcl::PointXYZHSV hsv_gt;	//Point of ground truth
				pcl::PointXYZHSV hsv_test;	//Point of classified cloud
				pcl::PointXYZRGB rgb_gt;
				pcl::PointXYZRGB rgb_test;
				rgb_gt = gt->points[*idx];
				rgb_test.rgb = *reinterpret_cast<float*>(&rgb);

				pcl::PointXYZRGBtoXYZHSV ( 	rgb_test, hsv_test);
				pcl::PointXYZRGBtoXYZHSV ( 	rgb_gt, hsv_gt);
				if((hsv_gt.v < 20) && (hsv_test.v < 20))
				{
					countCorrect++;
					countCorrectEdge++;
				}
				else if((hsv_test.h - hsv_gt.h) < 3)	//same color (independent from light conditions)
				{
					countCorrect++;
					switch(c_it->type)
					{
					case I_PLANE: countCorrectPlane++; break;
					case I_CONCAVE: countCorrectConc++; break;
					case I_CONVEX: countCorrectConv++; break;
					}
				}
				//std::cout <<"h gt: "  << hsv_gt.h << ", h classification: " << hsv_test.h << std::endl;
			}
		}
		else
		{
			std::cout << "cluster_type: " << c_it->type <<std::endl;
			countNoColorAssigned++;
		}
	}

	cv::imshow("classification",test_image);
	cv::waitKey(30);
	 */


	//precision
	//--------------------------------------------------------
	std::stringstream txt;

	compareImagesUsingColor(test_image, gt_color_image, c_p);

	struct percentages p_p = {0,0,0,0};
	p_p.conc = divide((float) c_p.countCorrectConc, (float) c_p.countConc);
	p_p.conv  = divide((float)c_p.countCorrectConv ,(float)c_p.countConv);
	p_p.edge  = divide((float)c_p.countCorrectEdge, (float) c_p.countEdge);
	p_p.plane = divide((float)c_p.countCorrectPlane, (float)c_p.countPlane);


	txt <<"\nPrecision:\n ------------------------\n";

	txt << "Overall number of points in cloud: " << gt->size() << std::endl;
	txt << "correctly classified points of type\n -plane:   \t" << c_p.countCorrectPlane <<" out of " <<c_p.countPlane <<"\n -concave:\t" << c_p.countCorrectConc <<" out of " <<c_p.countConc <<"\n -convex:\t" << c_p.countCorrectConv <<" out of " <<c_p.countConv <<"\n -edge: \t" << c_p.countCorrectEdge <<" out of " <<c_p.countEdge << std::endl;

	txt <<"Concave " << p_p.conc << " %\nConvex " << p_p.conv << " %\nEdge "<< p_p.edge << " %\nPlane " << p_p.plane << " %\n";



	//recall
	//----------------------------------------------------------

	compareImagesUsingColor(gt_color_image, test_image, c_r);

	struct percentages p_r = {0,0,0,0};
	p_r.conc  = divide((float)c_r.countCorrectConc, (float)c_r.countConc);
	p_r.conv  = divide((float)c_r.countCorrectConv, (float)c_r.countConv);
	p_r.edge  = divide((float)c_r.countCorrectEdge, (float)c_r.countEdge);
	p_r.plane = divide((float)c_r.countCorrectPlane, (float)c_r.countPlane);
	p_r.overall =  divide((float)c_r.countCorrect, (float)c_r.countCompared);

	txt <<"\nRecall:\n ------------------------\n";
	txt <<  "correctly classified points: " << c_r.countCorrect << " out of "<< c_r.countCompared << " compared points, that is " << p_r.overall <<"%\n";
	txt << "Overall number of points in cloud: " << gt->size() << std::endl;
	txt << "correctly classified points of type\n -plane:   \t" << c_r.countCorrectPlane <<" out of " <<c_r.countPlane <<"\n -concave:\t" << c_r.countCorrectConc <<" out of " <<c_r.countConc <<"\n -convex:\t" << c_r.countCorrectConv <<" out of " <<c_r.countConv <<"\n -edge: \t" << c_r.countCorrectEdge <<" out of " <<c_r.countEdge << std::endl;

	txt <<"Concave " << p_r.conc << " %\nConvex " << p_r.conv << " %\nEdge "<< p_r.edge << " %\nPlane " << p_r.plane << " %\n";

	rec.saveText(txt.str(), "eval");
	rec.saveCloud(gt, "cloud");

	return 0;
}
