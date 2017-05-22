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
 * edge_detection.h
 *
 *  Created on: May 17, 2013
 *      Author: rmb-ce
 */

#ifndef EDGE_DETECTION_H_
#define EDGE_DETECTION_H_

// OpenCV
#include <opencv2/opencv.hpp>

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

	EdgeDetection ():
		stepThreshold_(0.05),
		offsetConcConv_(1.5),
		lineLength_(20),
		thinSizeHalf_(4),
		windowX_(600),
		windowY_(600),
		th_plane_(0.7),	//0.7
		th_edge_(-0.4)	//-0.4
	{};

	inline void setStepThreshold(float th)
	{
		stepThreshold_ = th;
	}
	inline void setOffsetConcConv(float th)
	{
		offsetConcConv_ = th;
	}
	inline void setLineLength(int l)
	{
		lineLength_ = l;
	}
	inline void setWindowSize(int x, int y)
	{
		windowX_ = x;
		windowY_ = y;
	}

	void computeDepthEdges(cv::Mat depth_image, PointCloudInConstPtr pointcloud, cv::Mat& edgeImage, const float depth_factor=0.02f);

	void sobelLaplace(cv::Mat& color_image, cv::Mat& depth_image);


private:



	void coordinatesMat(cv::Mat& depth_image, PointCloudInConstPtr pointcloud, cv::Point2f dotIni, cv::Point2f dotEnd, cv::Mat& coordinates, bool& step);
	void approximateLineSVD(cv::Mat& depth_image, PointCloudInConstPtr pointcloud, cv::Point2f dotLeft, cv::Point2f dotRight, cv::Mat& abc, cv::Mat& coordinates, bool& step);
	void approximateLinePCA (cv::Mat& depth_image, PointCloudInConstPtr pointcloud, cv::Point2f dotIni, cv::Point2f dotEnd, cv::Mat& n, cv::Mat& coordinates, bool& step);
	void approximateLine(cv::Mat& depth_image, PointCloudInConstPtr pointcloud, cv::Point2f dotIni, cv::Point2f dotEnd, cv::Mat& abc);
	void scalarProduct(cv::Mat& abc1,cv::Mat& abc2,float& scalarProduct, bool& step);
	void curvatureType(cv::Mat& abc1,cv::Mat& abc2, int& concaveConvex);
	void approximateLineFullAndHalfDist (cv::Mat& depth_image, PointCloudInConstPtr pointcloud, cv::Point2f dotIni, cv::Point2f dotEnd, cv::Mat& abc);

	void thinEdges(cv::Mat& edgePicture, int xy);
	void drawLines(cv::Mat& plotXY, cv::Mat& coordinates, cv::Mat& abc);
	void printNeigh(PointCloudInConstPtr cloud, int x, int y);
	void drawLineAlongN(cv::Mat& plotZW, cv::Mat& coordinates, cv::Mat& n);

	void deriv2nd3pts (cv::Mat threePoints, float& deriv);
	void deriv2nd5pts (cv::Mat threePoints, float& deriv);
	void deriv2nd (cv::Mat depth_image,PointCloudInConstPtr cloud, cv::Point2f dotStart, cv::Point2f dotStop, float& deriv);





	float stepThreshold_;	//minimum distance which is detected as a step in depth coordinates
	float offsetConcConv_;	//how much the gradients need to differ
	int lineLength_;	//depth coordinates along two lines with length lineLength/2 are considered
	int thinSizeHalf_; //thinning of edges along line of length 2*thinSizeHalf_
	int windowX_;	//size of visualization windows in x-direction
	int windowY_;
	float th_plane_;	//threshold of scalarproduct. Only smaller values are taken into account for edges.
	float th_edge_;		//threshold of scalarproduct. Only larger values are taken into account for edges. Should be negative.

};


#include "cob_surface_classification/impl/edge_detection.hpp"


#endif /* EDGE_DETECTION_H_ */
