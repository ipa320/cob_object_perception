/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2014 \n
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
 * \date Date of creation: 14.05.2014
 *
 * \brief
 * functions detection 3d edges (border edges and surface discontinuities) in organized point cloud data
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

#ifndef EDGE_DETECTION_H_
#define EDGE_DETECTION_H_

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/pcl_base.h>

// timer
#include <iostream>
#include <vector>
#include "timer.h"


template <typename PointInT>
class EdgeDetection
{
public:

	typedef pcl::PointCloud<PointInT> PointCloudIn;
	typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

	EdgeDetection()
	: PI_FLOAT(3.14159265f), PIBY2_FLOAT(1.5707963f)
	{
		runtime_total_ = 0.;
		runtime_depth_image_ = 0.;
		runtime_sobel_ = 0.;	// image derivatives
		runtime_edge_ = 0.;
		runtime_visibility_ = 0.;
		runtime_normal_original_ = 0.;
		runtime_normal_edge_ = 0.;
		number_processed_images_ = 0;

		scan_line_width_ = 10;
	};

	void computeDepthEdges(PointCloudInConstPtr pointcloud, cv::Mat& edge, const float depth_factor)
	{
		// compute x,y, z images
		Timer tim;
		tim.start();
		cv::Mat x_image = cv::Mat::zeros(pointcloud->height, pointcloud->width, CV_32FC1);
		cv::Mat y_image = cv::Mat::zeros(pointcloud->height, pointcloud->width, CV_32FC1);
		cv::Mat z_image = cv::Mat::zeros(pointcloud->height, pointcloud->width, CV_32FC1);
		int i=0;
		for (unsigned int v=0; v<pointcloud->height; v++)
		{
			float* x_ptr = (float*)x_image.ptr(v);
			float* y_ptr = (float*)y_image.ptr(v);
			float* z_ptr = (float*)z_image.ptr(v);
			for (unsigned int u=0; u<pointcloud->width; u++, i++)
			{
				//matrix indices: row y, column x!
				const pcl::PointXYZRGB& point = pointcloud->at(u,v);

				if(point.z == point.z)	//test nan
				{
					*x_ptr = point.x;
					*y_ptr = point.y;
					*z_ptr = point.z;
					//z_image.at<float>(v,u) = point.z;
				}
				++x_ptr;
				++y_ptr;
				++z_ptr;
			}
		}
		//std::cout << "Time for x/y/z images: " << tim.getElapsedTimeInMilliSec() << "\n";
		runtime_depth_image_ += tim.getElapsedTimeInMilliSec();
		cv::Mat z_display;
		cv::normalize(z_image, z_display, 0, 1, cv::NORM_MINMAX);
		cv::imshow("depth", z_display);

		//visualization
//		cv::Mat depth_im_scaled;
//		cv::normalize(z_image, depth_im_scaled,0,1,cv::NORM_MINMAX);
//		cv::imshow("depth_image", depth_im_scaled);
//		cv::waitKey(10);


		//int key = 0;

		Timer total;
		total.start();
		tim.start();
		cv::Mat x_dx, y_dy, z_dx, z_dy;
		//cv::medianBlur(z_image, z_image, 5);
		const int kernel_size = 3; 					//  3		    5
		const double kernel_scale = 1./8.;			// 1./8.	1./(6.*16.)
		cv::Sobel(x_image, x_dx, -1, 1, 0, kernel_size, kernel_scale);
		cv::Sobel(y_image, y_dy, -1, 0, 1, kernel_size, kernel_scale);
		cv::Sobel(z_image, z_dx, -1, 1, 0, kernel_size, kernel_scale);
		cv::Sobel(z_image, z_dy, -1, 0, 1, kernel_size, kernel_scale);
		//cv::medianBlur(z_dx, z_dx, 5);
		//cv::medianBlur(z_dy, z_dy, 5);
		const int kernel_size2 = 7;
//		cv::Mat temp = z_dx.clone();
//		cv::bilateralFilter(temp, z_dx, 5, 90, 90);
//		temp = z_dy.clone();
//		cv::bilateralFilter(temp, z_dy, 5, 90, 90);
		cv::GaussianBlur(z_dx, z_dx, cv::Size(kernel_size2,kernel_size2), 0, 0);
		cv::GaussianBlur(z_dy, z_dy, cv::Size(kernel_size2,kernel_size2), 0, 0);
		//std::cout << "Time for slope Sobel: " << tim.getElapsedTimeInMilliSec() << "\n";
		runtime_sobel_ += tim.getElapsedTimeInMilliSec();

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
		edge = cv::Mat::zeros(z_image.rows, z_image.cols, CV_8UC1);
		const int max_line_width = 20;
		for (int v = max_line_width; v < z_dx.rows - max_line_width - 1; ++v)
		{
			for (int u = max_line_width; u < z_dx.cols - max_line_width - 1; ++u)
			{
				float depth = z_image.at<float>(v, u);
				if (depth==0.f)
					continue;
				float edge_threshold = std::max(0.0f, depth_factor*depth);
				if (z_dx.at<float>(v, u) <= -edge_threshold || z_dx.at<float>(v, u) >= edge_threshold ||
						z_dy.at<float>(v, u) <= -edge_threshold || z_dy.at<float>(v, u) >= edge_threshold)
					//edge.at<uchar>(v, u) = 254;
					edge.at<uchar>(v, u) = (uchar)std::min<float>(255.f, 50.f*(1.+sqrt(z_dx.at<float>(v, u)*z_dx.at<float>(v, u) + z_dy.at<float>(v, u)*z_dy.at<float>(v, u))));
			}
		}
		nonMaximumSuppression<uchar>(edge, z_dx, z_dy);
		cv::Mat edge_integral;
		cv::integral(edge, edge_integral, CV_32S);
//		std::cout << "Time for edge+integral: " << tim.getElapsedTimeInMilliSec() << "\n";

		int key = cv::waitKey(20);
		if (key == 'k')
			scan_line_width_ -= 5;
		if (key == 'l')
			scan_line_width_ += 5;
		std::cout << "scan_line_width = " << scan_line_width_ << std::endl;
		//cv::Mat angle_image = cv::Mat::zeros(edge.rows, edge.cols, CV_32FC1);

		// surface discontinuities
		const float min_detectable_edge_angle = 45.; //35.;	// minimum angle between two planes to consider their intersection an edge, measured in [° degree]
		// x lines
		//int scan_line_width_ = 10; // width of scan line left or right of a query pixel, measured in [px]
		int last_line_width = scan_line_width_;
		cv::Mat x_dx_integralX, z_dx_integralX;
		computeIntegralImageX(x_dx, x_dx_integralX, z_dx, z_dx_integralX);
		for (int v = max_line_width; v < z_dx.rows - max_line_width - 1; ++v)
		{
			int edge_start_index = -1;
			float max_edge_diff = 0;
			for (int u = max_line_width; u < z_dx.cols - max_line_width - 1; ++u)
			{
				if (v==240 && u==320)
					drawCoordinateSample(u, v, x_image, x_dx, y_image, y_dy, z_image, z_dx);

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
					// depth dependent scan line width for slope computation (1px width per 0.10m depth)
					scan_line_width_ = std::min(int(6.666*depth+1.666), max_line_width);
					if (scan_line_width_ <= 5)
						scan_line_width_ = last_line_width;
					else
						last_line_width = scan_line_width_;
					// do not compute if a depth discontinuity is on the line (ATTENTION: opencv uses a different indexing scheme which is basically +1 in x and y direction, so pixel itself is not included in sum)
					if (edge_integral.at<int>(v+1,u+scan_line_width_+1)-edge_integral.at<int>(v+1,u-scan_line_width_)-edge_integral.at<int>(v,u+scan_line_width_+1)+edge_integral.at<int>(v,u-scan_line_width_) != 0)
					{
						edge_start_index = -1;
						max_edge_diff = 0.f;
						continue;
					}

					// get average differences in x and z direction (ATTENTION: the integral images provide just the sum, not divided by number of elements, however, further processing only needs the sum, not the real average)
					// remark: the indexing of the integral image here differs from the OpenCV definition (here: the value a cell is included in the sum of the integral image's cell)
					double avg_dx_l = x_dx_integralX.at<float>(v, u-1) - x_dx_integralX.at<float>(v, u-scan_line_width_);
					double avg_dz_l = z_dx_integralX.at<float>(v, u-1) - z_dx_integralX.at<float>(v, u-scan_line_width_);
					float avg_dx_r = x_dx_integralX.at<float>(v, u+scan_line_width_) - x_dx_integralX.at<float>(v, u+1);
					float avg_dz_r = z_dx_integralX.at<float>(v, u+scan_line_width_) - z_dx_integralX.at<float>(v, u+1);

					// estimate angle difference
					float alpha_left = fast_atan2f_1(-avg_dz_l, -avg_dx_l);
					float alpha_right = fast_atan2f_1(avg_dz_r, avg_dx_r);
					float diff = fabs(alpha_left - alpha_right);
					if (diff!=0 && (diff < (180.-min_detectable_edge_angle) / 180. * CV_PI || diff > (180.+min_detectable_edge_angle) / 180. * CV_PI))
					{
						if (edge_start_index == -1)
							edge_start_index = u;
						float dist = fabs(CV_PI - diff);
						//angle_image.at<float>(v,u) += CV_PI - (alpha_left - alpha_right + (alpha_left<0. ? 2*CV_PI : 0));
						if (dist > max_edge_diff)
						{
							max_edge_diff = dist;
							edge_start_index = u;
						}
					}
					else
					{
						if (edge_start_index != -1)
						{
							//edge.at<uchar>(v, (edge_start_index+u-1)/2) = 255;
							edge.at<uchar>(v, edge_start_index) = 255;
							edge_start_index = -1;
							max_edge_diff = 0;
						}
					}
//				}
			}
		}
		// y lines
		//scan_line_width_ = 10; // 1px/0.10m
		last_line_width = scan_line_width_;
		cv::Mat y_dy_integralY, z_dy_integralY;
		computeIntegralImageY(y_dy, y_dy_integralY, z_dy, z_dy_integralY);
		std::vector<int> edge_start_index(z_dy.cols, -1);
		std::vector<float> max_edge_diff(z_dy.cols, 0);
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
					scan_line_width_ = std::min(int(6.666*depth+1.666), max_line_width);
					if (scan_line_width_ < 5)
						scan_line_width_ = last_line_width;
					else
						last_line_width = scan_line_width_;
					// do not compute if a depth discontinuity is on the line (ATTENTION: opencv uses a different indexing scheme which is basically +1 in x and y direction, so pixel itself is not included in sum)
					if (edge_integral.at<int>(v+scan_line_width_+1,u+1)-edge_integral.at<int>(v-scan_line_width_,u+1)-edge_integral.at<int>(v+scan_line_width_+1,u)+edge_integral.at<int>(v-scan_line_width_,u) != 0)
					{
						edge_start_index[u] = -1;
						max_edge_diff[u] = 0.f;
						continue;
					}

					// get average differences in x and z direction (ATTENTION: the integral images provide just the sum, not divided by number of elements, however, further processing only needs the sum, not the real average)
					double avg_dy_u = y_dy_integralY.at<float>(v-1, u) - y_dy_integralY.at<float>(v-scan_line_width_, u);
					double avg_dz_u = z_dy_integralY.at<float>(v-1, u) - z_dy_integralY.at<float>(v-scan_line_width_, u);
					float avg_dy_l = y_dy_integralY.at<float>(v+scan_line_width_, u) - y_dy_integralY.at<float>(v+1, u);
					float avg_dz_l = z_dy_integralY.at<float>(v+scan_line_width_, u) - z_dy_integralY.at<float>(v+1, u);

					// estimate angle difference
					float alpha_upper = fast_atan2f_1(-avg_dz_u, -avg_dy_u);
					float alpha_lower = fast_atan2f_1(avg_dz_l, avg_dy_l);
					float diff = fabs(alpha_upper - alpha_lower);
					if (diff!=0 && (diff < (180.-min_detectable_edge_angle) / 180. * CV_PI || diff > (180.+min_detectable_edge_angle) / 180. * CV_PI))
					{
						if (edge_start_index[u] == -1)
							edge_start_index[u] = v;
						float dist = fabs(CV_PI - diff);
						//angle_image.at<float>(v,u) += CV_PI - (alpha_upper - alpha_lower + (alpha_upper<0. ? 2*CV_PI : 0));
						if (dist > max_edge_diff[u])
						{
							max_edge_diff[u] = dist;
							edge_start_index[u] = v;
						}
					}
					else
					{
						if (edge_start_index[u] != -1)
						{
							//edge.at<uchar>((edge_start_index[u]+v-1)/2, u) = 255;
							edge.at<uchar>(edge_start_index[u], u) = 255;
							edge_start_index[u] = -1;
							max_edge_diff[u] = 0;
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
		//std::cout << "Time for slope+edge: " << tim.getElapsedTimeInMilliSec() << "\n";
		runtime_edge_ += tim.getElapsedTimeInMilliSec();

		// remaining problems:
		// 1. some edges = double edges
		// 2. noise -> speckle filter in the beginning?

		runtime_total_ += total.getElapsedTimeInMilliSec();
		++number_processed_images_;

//		std::cout << "runtime_total: " << runtime_total_/(double)number_processed_images_ <<
//					"\nruntime_depth_image: " << runtime_depth_image_/(double)number_processed_images_ <<
//					"\nruntime_sobel: " << runtime_sobel_/(double)number_processed_images_ <<
//					"\nruntime_edge: " << runtime_edge_/(double)number_processed_images_ << std::endl;
					//"\nruntime_visibility: " << runtime_visibility_/(double)number_processed_images_ <<

//		cv::imshow("z_dx", z_dx);
//		cv::normalize(x_dx, x_dx, 0., 1., cv::NORM_MINMAX);
//		cv::imshow("x_dx", x_dx);
		//cv::normalize(average_slope, average_slope, 0., 1., cv::NORM_MINMAX);
//		average_dz_right = average_dz_right * 15 + 0.5;
//		cv::imshow("average_slope", average_dz_right);
//		cv::imshow("edge", edge);

//		cv::Mat a_dx, a_dy;
//		cv::pow(angle_image, 2, angle_image);
//		cv::Sobel(angle_image, a_dx, -1, 1, 0, kernel_size, kernel_scale);
//		cv::Sobel(angle_image, a_dy, -1, 0, 1, kernel_size, kernel_scale);
//		nonMaximumSuppression<float>(angle_image, a_dx, a_dy);
//		cv::normalize(angle_image, angle_image, 0., 1., cv::NORM_MINMAX);
//		cv::imshow("angle image", angle_image);
	}

	template <typename t>
	void nonMaximumSuppression(cv::Mat& edge, const cv::Mat& dx, const cv::Mat& dy)
	{
		for (int v=1; v<edge.rows-1; ++v)
		{
			for (int u=1; u<edge.cols-1; ++u)
			{
				if (edge.at<t>(v,u)==0)
					continue;
				double x=dx.at<float>(v,u);
				double y=dy.at<float>(v,u);
				if (x==0 && y==0)
					continue;
				double mag = sqrt(x*x+y*y);
				x = floor(x/mag+0.5);
				y = floor(y/mag+0.5);
				t& edge_val = edge.at<t>(v,u);
				if (edge_val>edge.at<t>(v+y,u+x) && edge_val>edge.at<t>(v-y,u-x))
					edge_val = (t)254;
				else
					edge_val = (t)0;
			}
		}
	}

	void drawCoordinateSample(int u, int v, const cv::Mat& x_image, const cv::Mat& x_dx, const cv::Mat& y_image, const cv::Mat& y_dy, const cv::Mat& z_image, const cv::Mat& z_dx)
	{
		cv::Mat display = cv::Mat::zeros(600, 600, CV_8UC1);
		cv::line(display, cv::Point(0,display.rows/2), cv::Point(display.cols-1,display.rows/2), cv::Scalar(32), 1);

		float avg_x = cv::mean(x_image(cv::Rect(u-scan_line_width_,v,2*scan_line_width_+1,1))).val[0];
		float avg_dx = cv::mean(x_dx(cv::Rect(u-scan_line_width_,v,2*scan_line_width_+1,1))).val[0];
		float avg_y = cv::mean(y_image(cv::Rect(u,v-scan_line_width_,1,2*scan_line_width_+1))).val[0];
		float avg_zx = cv::mean(z_image(cv::Rect(u-scan_line_width_,v,2*scan_line_width_+1,1))).val[0];
		float avg_zy = cv::mean(z_image(cv::Rect(u,v-scan_line_width_,1,2*scan_line_width_+1))).val[0];
		float avg_dz = cv::mean(z_dx(cv::Rect(u-scan_line_width_,v,2*scan_line_width_+1,1))).val[0];
		int draw_u = 10;
		for (int du=-scan_line_width_; du<scan_line_width_; ++du, draw_u+=10)
		{
			const float scale1 = 1000.f;
			const float scale2 = 10000.f;
			float x = display.cols/2 + scale1*(x_image.at<float>(v, u+du) - avg_x);
			float dx = display.rows/2 + scale2*(x_dx.at<float>(v, u+du) - avg_dx);
			float y = display.cols/2 + scale1*(y_image.at<float>(v+du, u) - avg_y);
			float zx = display.rows/2 + scale1*(z_image.at<float>(v, u+du) - avg_zx);
			float dz = display.rows/2 + scale2*(z_dx.at<float>(v, u+du) - avg_dz);
			float zy = display.rows/2 + scale1*(z_image.at<float>(v+du, u) - avg_zy);
			float x2 = display.cols/2 + scale1*(x_image.at<float>(v, u+du+1) - avg_x);
			float dx2 = display.rows/2 + scale2*(x_dx.at<float>(v, u+du+1) - avg_dx);
			float y2 = display.cols/2 + scale1*(y_image.at<float>(v+du+1, u) - avg_y);
			float zx2 = display.rows/2 + scale1*(z_image.at<float>(v, u+du+1) - avg_zx);
			float dz2 = display.rows/2 + scale2*(z_dx.at<float>(v, u+du+1) - avg_dz);
			float zy2 = display.rows/2 + scale1*(z_image.at<float>(v+du+1, u) - avg_zy);
			cv::line(display, cv::Point(x,zx), cv::Point(x2,zx2), cv::Scalar(255), 1);
			cv::line(display, cv::Point(y,zy), cv::Point(y2,zy2), cv::Scalar(128), 1);
			if (du==0)
			{
				cv::circle(display, cv::Point(x,zx), 2, cv::Scalar(192));
				cv::circle(display, cv::Point(y,zy), 2, cv::Scalar(128));
			}
			cv::line(display, cv::Point(draw_u,dx), cv::Point(draw_u+10,dx2), cv::Scalar(64), 1);
			cv::line(display, cv::Point(draw_u,dz), cv::Point(draw_u+10,dz2), cv::Scalar(128), 1);
		}
		cv::imshow("measure", display);
		cv::waitKey(10);
	}

	int getScanLineWidth()
	{
		return scan_line_width_;
	}

private:

	// from https://gist.github.com/volkansalma/2972237
	//  or  http://lists.apple.com/archives/perfoptimization-dev/2005/Jan/msg00051.html
	const float PI_FLOAT; // = 3.14159265f;
	const float PIBY2_FLOAT; // = 1.5707963f;
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
				if (*srcX_ptr > 0.f)	// only take data with increasing metric x-coordinate (Kinect sensor may sporadically yield decreasing x coordinate with increasing u image coordinate)
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

	double runtime_total_;
	double runtime_depth_image_;
	double runtime_sobel_;	// image derivatives
	double runtime_edge_;
	double runtime_visibility_;
	double runtime_normal_original_;
	double runtime_normal_edge_;
	int number_processed_images_;

	int scan_line_width_;	// width of scan line left or right of a query pixel, measured in [px]
};


//#include "cob_surface_classification/impl/edge_detection.hpp"


#endif // EDGE_DETECTION_H_
