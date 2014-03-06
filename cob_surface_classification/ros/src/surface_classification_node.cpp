/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2013 \n+
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

/*switches for execution of processing steps*/

#define RECORD_MODE					false		//save color image and cloud for usage in EVALUATION_OFFLINE_MODE
#define COMPUTATION_MODE			true		//computations without record
#define EVALUATION_OFFLINE_MODE		false		//evaluation of stored pointcloud and image
#define EVALUATION_ONLINE_MODE		false		//computations plus evaluation of current computations plus record of evaluation

//steps in computation/evaluation_online mode:

#define SEG 						false 	//segmentation + refinement
#define SEG_WITHOUT_EDGES 			false 	//segmentation without considering edge image (wie Steffen)
#define SEG_REFINE					false 	//segmentation refinement according to curvatures (outdated)
#define CLASSIFY 					false	//classification


#define NORMAL_VIS 					true 	//visualisation of normals
#define SEG_VIS 					false 	//visualisation of segmentation
#define SEG_WITHOUT_EDGES_VIS 		false 	//visualisation of segmentation without edge image
#define CLASS_VIS 					false 	//visualisation of classification




// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

// topics
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// boost
#include <boost/bind.hpp>

// point cloud
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>


//internal includes
#include <cob_surface_classification/edge_detection.h>
//#include <cob_surface_classification/surface_classification.h>
#include <cob_surface_classification/organized_normal_estimation.h>
#include <cob_surface_classification/refine_segmentation.h>

//package includes
#include <cob_3d_segmentation/depth_segmentation.h>
#include <cob_3d_segmentation/cluster_classifier.h>
#include <cob_3d_mapping_common/point_types.h>


//records
#include "scene_recording.h"
//evaluation
#include "Evaluation.h"



class SurfaceClassificationNode
{
public:
	typedef cob_3d_segmentation::PredefinedSegmentationTypes ST;

	SurfaceClassificationNode(ros::NodeHandle nh)
	: node_handle_(nh)
	{
		it_ = 0;
		sync_input_ = 0;

		it_ = new image_transport::ImageTransport(node_handle_);
		colorimage_sub_.subscribe(*it_, "colorimage_in", 1);
		pointcloud_sub_.subscribe(node_handle_, "pointcloud_in", 1);

		sync_input_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >(30);
		sync_input_->connectInput(colorimage_sub_, pointcloud_sub_);
		sync_input_->registerCallback(boost::bind(&SurfaceClassificationNode::inputCallback, this, _1, _2));
	}

	~SurfaceClassificationNode()
	{
		if (it_ != 0)
			delete it_;
		if (sync_input_ != 0)
			delete sync_input_;
	}


	// Converts a color image message to cv::Mat format.
	void convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
	{
		try
		{
			image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("PeopleDetection: cv_bridge exception: %s", e.what());
		}
		image = image_ptr->image;
	}

	// from https://gist.github.com/volkansalma/2972237
	//  or  http://lists.apple.com/archives/perfoptimization-dev/2005/Jan/msg00051.html
	#define PI_FLOAT     3.14159265f
	#define PIBY2_FLOAT  1.5707963f
	// |error| < 0.005
	float fast_atan2f_1(float y, float x)
	{
		if (x == 0.0f)
		{
			if (y > 0.0f) return PIBY2_FLOAT;
			if (y == 0.0f) return 0.0f;
			return -PIBY2_FLOAT;
		}
		float atan;
		float z = y/x;
		if (fabsf(z) < 1.0f)
		{
			atan = z/(1.0f + 0.28f*z*z);
			if (x < 0.0f)
			{
				if (y < 0.0f) return atan - PI_FLOAT;
				return atan + PI_FLOAT;
			}
		}
		else
		{
			atan = PIBY2_FLOAT - z/(z*z + 0.28f);
			if ( y < 0.0f ) return atan - PI_FLOAT;
		}
		return atan;
	}

	float fast_atan2f_2(float y, float x)
	{
		//http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
		//Volkan SALMA

		const float ONEQTR_PI = M_PI / 4.0;
		const float THRQTR_PI = 3.0 * M_PI / 4.0;
		float r, angle;
		float abs_y = fabs(y) + 1e-10f; // kludge to prevent 0/0 condition
		if (x < 0.0f)
		{
			r = (x + abs_y) / (abs_y - x);
			angle = THRQTR_PI;
		}
		else
		{
			r = (x - abs_y) / (x + abs_y);
			angle = ONEQTR_PI;
		}
		angle += (0.1963f * r * r - 0.9817f) * r;
		if (y < 0.0f)
			return (-angle); // negate if in quad III or IV
		else
			return (angle);
	}

	float fast_arccosf(float x)
	{
		float x2 = x*x;
		float x4 = x2*x2;
		return (CV_PI/2.0 - (x + 1./6.*x*x2 + 3./40.*x*x4));
	}

	// creates an integral image within x-direction (i.e. line-wise, horizontally) for two source images
	void computeIntegralImageX(const cv::Mat& srcX, cv::Mat& dstX, const cv::Mat& srcZ, cv::Mat& dstZ)
	{
		dstX = cv::Mat(srcX.rows, srcX.cols, CV_32FC1);
		dstZ = cv::Mat(srcX.rows, srcX.cols, CV_32FC1);
		for (int v=0; v<srcX.rows; ++v)
		{
			float* dstX_ptr = (float*)dstX.ptr(v);
			const float* srcX_ptr = (const float*)srcX.ptr(v);
			float* dstZ_ptr = (float*)dstZ.ptr(v);
			const float* srcZ_ptr = (const float*)srcZ.ptr(v);
			float sumX = 0.f;
			float sumZ = 0.f;
			for (int u=0; u<srcX.cols; ++u)
			{
				if (*srcX_ptr > 0.f)
				{
					sumX += *srcX_ptr;
					sumZ += *srcZ_ptr;
				}
				*dstX_ptr = sumX;
				srcX_ptr++;
				dstX_ptr++;
				*dstZ_ptr = sumZ;
				srcZ_ptr++;
				dstZ_ptr++;
			}
		}
	}

	// creates an integral image within y-direction (i.e. column-wise, vertically) for two source images
	void computeIntegralImageY(const cv::Mat& srcY, cv::Mat& dstY, const cv::Mat& srcZ, cv::Mat& dstZ)
	{
		dstY = cv::Mat(srcY.rows, srcY.cols, CV_32FC1);
		dstZ = cv::Mat(srcY.rows, srcY.cols, CV_32FC1);
		float* dstY_ptr = (float*)dstY.ptr(0);
		const float* srcY_ptr = (const float*)srcY.ptr(0);
		float* dstZ_ptr = (float*)dstZ.ptr(0);
		const float* srcZ_ptr = (const float*)srcZ.ptr(0);
		// copy first line
		for (int u=0; u<srcY.cols; ++u)
		{
			*dstY_ptr = *srcY_ptr;
			dstY_ptr++; srcY_ptr++;
			*dstZ_ptr = *srcZ_ptr;
			dstZ_ptr++; srcZ_ptr++;
		}
		// process remainder
		for (int v=1; v<srcY.rows; ++v)
		{
			float* dstY_ptr = (float*)dstY.ptr(v);
			float* dstYprev_ptr = (float*)dstY.ptr(v-1);
			const float* srcY_ptr = (const float*)srcY.ptr(v);
			float* dstZ_ptr = (float*)dstZ.ptr(v);
			float* dstZprev_ptr = (float*)dstZ.ptr(v-1);
			const float* srcZ_ptr = (const float*)srcZ.ptr(v);
			for (int u=0; u<srcY.cols; ++u)
			{
				if (*srcY_ptr > 0.f)
				{
					*dstY_ptr = *dstYprev_ptr + *srcY_ptr;
					*dstZ_ptr = *dstZprev_ptr + *srcZ_ptr;
				}
				else
				{
					*dstY_ptr = *dstYprev_ptr;
					*dstZ_ptr = *dstZprev_ptr;
				}
				srcY_ptr++;
				dstY_ptr++;
				dstYprev_ptr++;
				srcZ_ptr++;
				dstZ_ptr++;
				dstZprev_ptr++;
			}
		}
	}

	void inputCallback(const sensor_msgs::Image::ConstPtr& color_image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
	{

		ROS_INFO("Input Callback");

		// convert color image to cv::Mat
		cv_bridge::CvImageConstPtr color_image_ptr;
		cv::Mat color_image;
		convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::fromROSMsg(*pointcloud_msg, *cloud);
		if(cloud->height == 1 && cloud->points.size() == 307200)
		{
			cloud->height = 480;
			cloud->width = 640;
		}

		//compute depth_image: greyvalue represents depth z
		Timer tim;
		tim.start();
		cv::Mat x_image = cv::Mat::zeros(cloud->height, cloud->width, CV_32FC1);
		cv::Mat y_image = cv::Mat::zeros(cloud->height, cloud->width, CV_32FC1);
		cv::Mat z_image = cv::Mat::zeros(cloud->height, cloud->width, CV_32FC1);
		int i=0;
		for (unsigned int v=0; v<cloud->height; v++)
		{
			for (unsigned int u=0; u<cloud->width; u++, i++)
			{
				//matrix indices: row y, column x!
				pcl::PointXYZRGB& point = cloud->at(u,v);
				if(point.z == point.z)	//test nan
				{
					x_image.at<float>(v,u) = point.x;
					y_image.at<float>(v,u) = point.y;
					z_image.at<float>(v,u) = point.z;
				}
			}
		}
		std::cout << "Time for x/y/z images: " << tim.getElapsedTimeInMilliSec() << "\n";

		//visualization
		cv::Mat depth_im_scaled;
		cv::normalize(z_image, depth_im_scaled,0,1,cv::NORM_MINMAX);
		cv::imshow("depth_image", depth_im_scaled);
//		cv::waitKey(10);

		//draw crossline
		//int lineLength = 20;
		//cv::line(color_image,cv::Point2f(2*depth_image.cols/3 -lineLength/2, 2*depth_image.rows/3),cv::Point2f(2*depth_image.cols/3 +lineLength/2, 2*depth_image.rows/3),CV_RGB(0,1,0),1);
		//cv::line(color_image,cv::Point2f(2*depth_image.cols/3 , 2*depth_image.rows/3 +lineLength/2),cv::Point2f(2*depth_image.cols/3 , 2*depth_image.rows/3 -lineLength/2),CV_RGB(0,1,0),1);



		//record scene
		//----------------------------------------
		if(RECORD_MODE)
		{
			cv::Mat im_flipped;
			cv::flip(color_image, im_flipped,-1);
			cv::imshow("image", im_flipped);
			int key = cv::waitKey(50);
			//record if "r" is pressed while "image"-window is activated
			if(key == 1048690)
			{
				rec_.saveImage(im_flipped,"color");
				rec_.saveCloud(cloud,"cloud");
			}

		}

		//----------------------------------------

		int key = 0;
		cv::imshow("image", color_image);
		if(!EVALUATION_ONLINE_MODE)
			cv::waitKey(10);
		//if(EVALUATION_ONLINE_MODE){ key = cv::waitKey(50);}

		tim.start();
		cv::Mat x_dx, y_dy, z_dx, z_dy;
		//cv::medianBlur(z_image, z_image, 5);
		cv::Sobel(x_image, x_dx, -1, 1, 0, 5, 1./(6.*16.));
		cv::Sobel(y_image, y_dy, -1, 0, 1, 5, 1./(6.*16.));
		cv::Sobel(z_image, z_dx, -1, 1, 0, 5, 1./(6.*16.));
		cv::Sobel(z_image, z_dy, -1, 0, 1, 5, 1./(6.*16.));
		cv::medianBlur(z_dx, z_dx, 5);
		cv::medianBlur(z_dy, z_dy, 5);
		std::cout << "Time for slope Sobel: " << tim.getElapsedTimeInMilliSec() << "\n";
		tim.start();
//		cv::Mat kx, ky;
//		cv::getDerivKernels(kx, ky, 1, 0, 5, false, CV_32F);
//		std::cout << "kx:\n";
//		for (int i=0; i<kx.rows; ++i)
//		{
//			for (int j=0; j<kx.cols; ++j)
//				std::cout << kx.at<float>(i,j) << "\t";
//			std::cout << std::endl;
//		}
//		std::cout << "\nky:\n";
//		for (int i=0; i<ky.rows; ++i)
//		{
//			for (int j=0; j<ky.cols; ++j)
//				std::cout << ky.at<float>(i,j) << "\t";
//			std::cout << std::endl;
//		}

		// depth discontinuities
		cv::Mat edge = cv::Mat::zeros(z_image.rows, z_image.cols, CV_8UC1);
		const int max_line_width = 30;
		for (int v = max_line_width; v < z_dx.rows - max_line_width - 1; ++v)
		{
			for (int u = max_line_width; u < z_dx.cols - max_line_width - 1; ++u)
			{
				float depth = z_image.at<float>(v, u);
				if (depth==0.f)
					continue;
				if (z_dx.at<float>(v, u) <= -0.02*depth || z_dx.at<float>(v, u) >= 0.02*depth ||
						z_dy.at<float>(v, u) <= -0.02*depth || z_dy.at<float>(v, u) >= 0.02*depth)
					edge.at<uchar>(v, u) = 255;
			}
		}
		cv::Mat edge_integral;
		cv::integral(edge, edge_integral, CV_32S);
		std::cout << "Time for edge+integral: " << tim.getElapsedTimeInMilliSec() << "\n";

		// surface discontinuities
		// x lines
		int line_width = 10; // 1px/0.10m
		int last_line_width = 10;
		cv::Mat x_dx_integralX, z_dx_integralX;
		computeIntegralImageX(x_dx, x_dx_integralX, z_dx, z_dx_integralX);
		for (int v = max_line_width; v < z_dx.rows - max_line_width - 1; ++v)
		{
			int edge_start_index = -1;
			for (int u = max_line_width; u < z_dx.cols - max_line_width - 1; ++u)
			{
				float depth = z_image.at<float>(v, u);
				if (depth==0.f)
					continue;
				//if (z_dx.at<float>(v, u) <= -0.05 || z_dx.at<float>(v, u) >= 0.05)
//				if (z_dx.at<float>(v, u) <= -0.02*depth || z_dx.at<float>(v, u) >= 0.02*depth)
//				{
//					edge.at<uchar>(v, u) = 255;
//				}
//				else
//				{
					// depth dependent scan line width for slope computation (1px/0.10m)
					line_width = std::min(int(10 * depth), max_line_width);
					if (line_width == 0)
						line_width = last_line_width;
					else
						last_line_width = line_width;
					// do not compute if a depth discontinuity is on the line
					if (edge_integral.at<int>(v,u+line_width)-edge_integral.at<int>(v,u-line_width)-edge_integral.at<int>(v-1,u+line_width)+edge_integral.at<int>(v-1,u-line_width) != 0)
						continue;

					// get average differences in x and z direction (ATTENTION: the integral images provide just the sum, not divided by number of elements, however, further processing only needs the sum, not the real average)
					double avg_dx_l = x_dx_integralX.at<float>(v, u-1) - x_dx_integralX.at<float>(v, u-line_width);
					double avg_dz_l = z_dx_integralX.at<float>(v, u-1) - z_dx_integralX.at<float>(v, u-line_width);
					float avg_dx_r = x_dx_integralX.at<float>(v, u+line_width) - x_dx_integralX.at<float>(v, u+1);
					float avg_dz_r = z_dx_integralX.at<float>(v, u+line_width) - z_dx_integralX.at<float>(v, u+1);

					// estimate angle difference
					float alpha_left = fast_atan2f_1(-avg_dz_l, -avg_dx_l);
					float alpha_right = fast_atan2f_1(avg_dz_r, avg_dx_r);
					float diff = fabs(alpha_left - alpha_right);
					if (diff!=0 && (diff < 145. / 180. * CV_PI || diff > 215. / 180. * CV_PI))
					{
						//edge.at<uchar>(v, u) = 32; //(diff < 145. / 180. * CV_PI ? -64 : 64);//64 + 64 * 2 * fabs(CV_PI - fabs(alpha_left - alpha_right)) / CV_PI;
						if (edge_start_index == -1)
							edge_start_index = u;
					}
					else
					{
						if (edge_start_index != -1)
						{
							edge.at<uchar>(v, (edge_start_index+u-1)/2) = 255;	//192
							edge_start_index = -1;
						}
					}
//				}
			}
		}
		// y lines
		line_width = 10; // 1px/0.10m
		last_line_width = 10;
		cv::Mat y_dy_integralY, z_dy_integralY;
		computeIntegralImageY(y_dy, y_dy_integralY, z_dy, z_dy_integralY);
		std::vector<int> edge_start_index(z_dy.cols, -1);
		for (int v = max_line_width; v < z_dy.rows - max_line_width - 1; ++v)
		{
			for (int u = max_line_width; u < z_dy.cols - max_line_width - 1; ++u)
			{
				float depth = z_image.at<float>(v, u);
				if (depth==0.f)
					continue;
				//if (z_dy.at<float>(v, u) <= -0.05 || z_dy.at<float>(v, u) >= 0.05)
//				if (z_dy.at<float>(v, u) <= -0.02*depth || z_dy.at<float>(v, u) >= 0.02*depth)
//				{
//					edge.at<uchar>(v, u) = 255;
//				}
//				else
				if (edge.at<uchar>(v, u) == 0)
				{
					// depth dependent scan line width for slope computation (1px/0.10m)
					line_width = std::min(int(10 * depth), max_line_width);
					if (line_width == 0)
						line_width = last_line_width;
					else
						last_line_width = line_width;
					// do not compute if a depth discontinuity is on the line
					if (edge_integral.at<int>(v+line_width,u)-edge_integral.at<int>(v-line_width,u)-edge_integral.at<int>(v+line_width,u-1)+edge_integral.at<int>(v-line_width,u-1) != 0)
						continue;

					// get average differences in x and z direction (ATTENTION: the integral images provide just the sum, not divided by number of elements, however, further processing only needs the sum, not the real average)
					double avg_dy_u = y_dy_integralY.at<float>(v-1, u) - y_dy_integralY.at<float>(v-line_width, u);
					double avg_dz_u = z_dy_integralY.at<float>(v-1, u) - z_dy_integralY.at<float>(v-line_width, u);
					float avg_dy_l = y_dy_integralY.at<float>(v+line_width, u) - y_dy_integralY.at<float>(v+1, u);
					float avg_dz_l = z_dy_integralY.at<float>(v+line_width, u) - z_dy_integralY.at<float>(v+1, u);

					// estimate angle difference
					float alpha_upper = fast_atan2f_1(-avg_dz_u, -avg_dy_u);
					float alpha_lower = fast_atan2f_1(avg_dz_l, avg_dy_l);
					float diff = fabs(alpha_upper - alpha_lower);
					if (diff!=0 && (diff < 145. / 180. * CV_PI || diff > 215. / 180. * CV_PI))
					{
						//edge.at<uchar>(v, u) = 32; //128 + (diff < 145. / 180. * CV_PI ? -64 : 64);//64 + 64 * 2 * fabs(CV_PI - fabs(alpha_left - alpha_right)) / CV_PI;
						if (edge_start_index[u] == -1)
							edge_start_index[u] = v;
					}
					else
					{
						if (edge_start_index[u] != -1)
						{
							edge.at<uchar>((edge_start_index[u]+v-1)/2, u) = 255;  //192;
							edge_start_index[u] = -1;
						}
					}
				}
			}
		}
		cv::dilate(edge, edge, cv::Mat(), cv::Point(-1,-1), 1);
		cv::erode(edge, edge, cv::Mat(), cv::Point(-1,-1), 1);
		for (int v=0; v<z_image.rows; ++v)
			for (int u=0; u<z_image.cols; ++u)
				if (z_image.at<float>(v,u)==0)
					edge.at<uchar>(v,u)=0;
		std::cout << "Time for slope+edge: " << tim.getElapsedTimeInMilliSec() << "\n";

		// remaining problems:
		// 1. some edges = double edges
		// 2. noise -> speckle filter in the beginning?

//		cv::imshow("z_dx", z_dx);
//		cv::normalize(x_dx, x_dx, 0., 1., cv::NORM_MINMAX);
//		cv::imshow("x_dx", x_dx);
		//cv::normalize(average_slope, average_slope, 0., 1., cv::NORM_MINMAX);
//		average_dz_right = average_dz_right * 15 + 0.5;
//		cv::imshow("average_slope", average_dz_right);
		cv::imshow("edge", edge);
		cv::waitKey(10);
		return;


		//std::cout << key <<endl;
		//record if "e" is pressed while "image"-window is activated
		if(COMPUTATION_MODE || (EVALUATION_ONLINE_MODE && key == 1048677))
		{
			pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			pcl::PointCloud<pcl::Normal>::Ptr normalsWithoutEdges(new pcl::PointCloud<pcl::Normal>);
			pcl::PointCloud<PointLabel>::Ptr labels(new pcl::PointCloud<PointLabel>);
			pcl::PointCloud<PointLabel>::Ptr labelsWithoutEdges(new pcl::PointCloud<PointLabel>);
			ST::Graph::Ptr graph(new ST::Graph);
			ST::Graph::Ptr graphWithoutEdges(new ST::Graph);

/*
			tim.start();
			oneWithoutEdges_.setInputCloud(cloud);
			oneWithoutEdges_.setPixelSearchRadius(8,1,1);
			oneWithoutEdges_.setOutputLabels(labelsWithoutEdges);
			oneWithoutEdges_.setSkipDistantPointThreshold(8);	//PUnkte mit einem Abstand in der Tiefe von 8 werden nicht mehr zur Nachbarschaft gezählt
			oneWithoutEdges_.compute(*normalsWithoutEdges);
			std::cout << "Normal computation without edges: " << tim.getElapsedTimeInMilliSec() << "\n";
/*/
			cv::Mat edgeImage = cv::Mat::ones(z_image.rows,z_image.cols,CV_32FC1);
			for (int v=0; v<edge.rows; ++v)
				for (int u=0; u<edge.cols; ++u)
					edgeImage.at<float>(v,u) = 255-edge.at<uchar>(v,u);
			//edge_detection_.computeDepthEdges(z_image, cloud, edgeImage);

			//edge_detection_.sobelLaplace(color_image,depth_image);

			//cv::imshow("edge_image", edgeImage);
			//cv::waitKey(10);

			//Timer timer;
			//timer.start();
			//for(int i=0; i<10; i++)
			//{

			tim.start();
			one_.setInputCloud(cloud);
			one_.setPixelSearchRadius(8,1,1);	//call before calling computeMaskManually()!!!
			one_.computeMaskManually_increasing(cloud->width);
			one_.setEdgeImage(edgeImage);
			one_.setOutputLabels(labels);
			one_.setSameDirectionThres(0.94);
			one_.setSkipDistantPointThreshold(8);	//don't consider points in neighbourhood with depth distance larger than 8
			one_.compute(*normals);
			std::cout << "Normal computation: " << tim.getElapsedTimeInMilliSec() << "\n";

			//}timer.stop();
			//std::cout << timer.getElapsedTimeInMilliSec() << " ms for normalEstimation on the whole image, averaged over 10 iterations\n";
//*/


			if(NORMAL_VIS)
			{
				// visualize normals
				pcl::visualization::PCLVisualizer viewerNormals("Cloud and Normals");
				viewerNormals.setBackgroundColor (0.0, 0.0, 0);
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbNormals(cloud);

				viewerNormals.addPointCloud<pcl::PointXYZRGB> (cloud, rgbNormals, "cloud");
				viewerNormals.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, normals,2,0.005,"normals");
				//viewerNormals.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, normalsWithoutEdges,2,0.005,"normalsWithoutEdges");
				viewerNormals.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

				while (!viewerNormals.wasStopped ())
				{
					viewerNormals.spinOnce();
				}
				viewerNormals.removePointCloud("cloud");
			}

			return;

			if(SEG || EVALUATION_ONLINE_MODE)
			{
				seg_.setInputCloud(cloud);
				seg_.setNormalCloudIn(normals);
				seg_.setLabelCloudInOut(labels);
				seg_.setClusterGraphOut(graph);
				seg_.performInitialSegmentation();
				seg_.refineSegmentation();
			}
			if(SEG_WITHOUT_EDGES)
			{
				segWithoutEdges_.setInputCloud(cloud);
				segWithoutEdges_.setNormalCloudIn(normalsWithoutEdges);
				segWithoutEdges_.setLabelCloudInOut(labelsWithoutEdges);
				segWithoutEdges_.setClusterGraphOut(graphWithoutEdges);
				segWithoutEdges_.performInitialSegmentation();
			}

			if(SEG_VIS)
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented(new pcl::PointCloud<pcl::PointXYZRGB>);
				*segmented = *cloud;
				graph->clusters()->mapClusterColor(segmented);

				// visualize segmentation
				pcl::visualization::PCLVisualizer viewer("segmentation");
				viewer.setBackgroundColor (0.0, 0.0, 0);
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(segmented);
				viewer.addPointCloud<pcl::PointXYZRGB> (segmented,rgb,"seg");
				while (!viewer.wasStopped ())
				{
					viewer.spinOnce();
				}
				viewer.removePointCloud("seg");
			}
			if(SEG_WITHOUT_EDGES_VIS)
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedWithoutEdges(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::copyPointCloud<pcl::PointXYZRGB,pcl::PointXYZRGB>(*cloud, *segmentedWithoutEdges);
				graphWithoutEdges->clusters()->mapClusterColor(segmentedWithoutEdges);

				pcl::visualization::PCLVisualizer viewerWithoutEdges("segmentationWithoutEdges");

				viewerWithoutEdges.setBackgroundColor (0.0, 0.0, 0);
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbWithoutEdges(segmentedWithoutEdges);
				viewerWithoutEdges.addPointCloud<pcl::PointXYZRGB> (segmentedWithoutEdges,rgbWithoutEdges,"segWithoutEdges");
				while (!viewerWithoutEdges.wasStopped ())
				{
					viewerWithoutEdges.spinOnce();
				}
			}
			if(SEG_REFINE)
			{
				//merge segments with similar curvature characteristics
				segRefined_.setInputCloud(cloud);
				segRefined_.setClusterGraphInOut(graph);
				segRefined_.setLabelCloudInOut(labels);
				segRefined_.setNormalCloudIn(normals);
				//segRefined_.setCurvThres()
				segRefined_.refineUsingCurvature();
				//segRefined_.printCurvature(color_image);

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedRef(new pcl::PointCloud<pcl::PointXYZRGB>);
				*segmentedRef = *cloud;
				graph->clusters()->mapClusterColor(segmentedRef);

				// visualize refined segmentation
				pcl::visualization::PCLVisualizer viewerRef("segmentationRef");
				viewerRef.setBackgroundColor (0.0, 0.0, 0);
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbRef(segmentedRef);
				viewerRef.addPointCloud<pcl::PointXYZRGB> (segmentedRef,rgbRef,"segRef");

				while (!viewerRef.wasStopped ())
				{
					viewerRef.spinOnce();
				}
				viewerRef.removePointCloud("segRef");
			}



			if(CLASSIFY|| EVALUATION_ONLINE_MODE)
			{
				//classification

				cc_.setClusterHandler(graph->clusters());
				cc_.setNormalCloudInOut(normals);
				cc_.setLabelCloudIn(labels);
				cc_.setPointCloudIn(cloud);
				//cc_.setMaskSizeSmooth(14);
				cc_.classify();
			}
			if(CLASS_VIS)
			{

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr classified(new pcl::PointCloud<pcl::PointXYZRGB>);
				*classified = *cloud;
				graph->clusters()->mapTypeColor(classified);
				graph->clusters()->mapClusterBorders(classified);

				// visualize classification
				pcl::visualization::PCLVisualizer viewerClass("classification");
				viewerClass.setBackgroundColor (0.0, 0.0, 0);
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbClass(classified);
				viewerClass.addPointCloud<pcl::PointXYZRGB> (classified,rgbClass,"class");

				while (!viewerClass.wasStopped ())
				{
					viewerClass.spinOnce();
				}
				viewerClass.removePointCloud("class");
			}

			if(EVALUATION_ONLINE_MODE)
			{
				eval_.setClusterHandler(graph->clusters());
				eval_.compareClassification(cloud,color_image);
			}


		}
//		if(EVALUATION_OFFLINE_MODE)
//		{
//			TODO
//			std::string gt_filename = ...; //path to ground truth cloud
//			eval_.compareClassification(gt_filename);
//		}

	}//inputCallback()


private:
	ros::NodeHandle node_handle_;

	// messages
	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter colorimage_sub_; ///< Color camera image topic
	message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >* sync_input_;

	//records
	Scene_recording rec_;

	cob_features::OrganizedNormalEstimation<pcl::PointXYZRGB, pcl::Normal, PointLabel> one_;
	cob_features::OrganizedNormalEstimation<pcl::PointXYZRGB, pcl::Normal, PointLabel> oneWithoutEdges_;

	EdgeDetection<pcl::PointXYZRGB> edge_detection_;
	cob_3d_segmentation::DepthSegmentation<ST::Graph, ST::Point, ST::Normal, ST::Label> seg_;
	cob_3d_segmentation::RefineSegmentation<ST::Graph, ST::Point, ST::Normal, ST::Label> segRefined_;

	cob_3d_segmentation::DepthSegmentation<ST::Graph, ST::Point, ST::Normal, ST::Label> segWithoutEdges_;

	cob_3d_segmentation::ClusterClassifier<ST::CH, ST::Point, ST::Normal, ST::Label> cc_;

	//evaluation
	Evaluation eval_;

};

int main (int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "cob_surface_classification");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create and initialize an instance of CameraDriver
	SurfaceClassificationNode surfaceClassification(nh);

	ros::spin();

	return (0);
}
