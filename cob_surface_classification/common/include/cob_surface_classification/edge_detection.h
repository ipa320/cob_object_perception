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
	};

	void computeDepthEdges(PointCloudInConstPtr pointcloud, cv::Mat& edge)
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
		cv::Sobel(x_image, x_dx, -1, 1, 0, 5, 1./(6.*16.));
		cv::Sobel(y_image, y_dy, -1, 0, 1, 5, 1./(6.*16.));
		cv::Sobel(z_image, z_dx, -1, 1, 0, 5, 1./(6.*16.));
		cv::Sobel(z_image, z_dy, -1, 0, 1, 5, 1./(6.*16.));
		//cv::medianBlur(z_dx, z_dx, 5);
		//cv::medianBlur(z_dy, z_dy, 5);
		cv::GaussianBlur(z_dx, z_dx, cv::Size(5,5), 0, 0);
		cv::GaussianBlur(z_dy, z_dy, cv::Size(5,5), 0, 0);
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
		//cv::Mat edge = cv::Mat::zeros(z_image.rows, z_image.cols, CV_8UC1);
		edge = cv::Mat::zeros(z_image.rows, z_image.cols, CV_8UC1);
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
//		cv::Mat edge_integral;
//		cv::integral(edge, edge_integral, CV_32S);
		//std::cout << "Time for edge+integral: " << tim.getElapsedTimeInMilliSec() << "\n";

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
					// do not compute if a depth discontinuity is on the line (ATTENTION: opencv uses a different indexing scheme which is basically +1 in x and y direction, so pixel itself is not included in sum)
		//			if (edge_integral.at<int>(v+1,u+line_width+1)-edge_integral.at<int>(v+1,u-line_width)-edge_integral.at<int>(v,u+line_width+1)+edge_integral.at<int>(v,u-line_width) != 0)
		//				continue;

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
						//edge.at<uchar>(v, u) = 16; //(diff < 145. / 180. * CV_PI ? -64 : 64);//64 + 64 * 2 * fabs(CV_PI - fabs(alpha_left - alpha_right)) / CV_PI;
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
					// do not compute if a depth discontinuity is on the line (ATTENTION: opencv uses a different indexing scheme which is basically +1 in x and y direction, so pixel itself is not included in sum)
		//			if (edge_integral.at<int>(v+line_width+1,u+1)-edge_integral.at<int>(v-line_width,u+1)-edge_integral.at<int>(v+line_width+1,u)+edge_integral.at<int>(v-line_width,u) != 0)
		//todo:				continue;

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

	double runtime_total_;
	double runtime_depth_image_;
	double runtime_sobel_;	// image derivatives
	double runtime_edge_;
	double runtime_visibility_;
	double runtime_normal_original_;
	double runtime_normal_edge_;
	int number_processed_images_;

};


//#include "cob_surface_classification/impl/edge_detection.hpp"


#endif // EDGE_DETECTION_H_
