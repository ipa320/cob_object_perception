/*
 * edge_detection.h
 *
 *  Created on: May 17, 2013
 *      Author: rmb-ce
 */

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

	EdgeDetection ():
		stepThreshold_(0.05),
		offsetConcConv_(1.5),
		lineLength_(40),
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

	void computeDepthEdges(cv::Mat depth_image, PointCloudInConstPtr pointcloud, cv::Mat& edgeImage);

	void sobelLaplace(cv::Mat& color_image, cv::Mat& depth_image);


private:



	void coordinatesMat(cv::Mat& depth_image, PointCloudInConstPtr pointcloud, cv::Point2f dotIni, cv::Point2f dotEnd, cv::Mat& coordinates, bool& step);
	void approximateLine(cv::Mat& depth_image, PointCloudInConstPtr pointcloud, cv::Point2f dotLeft, cv::Point2f dotRight, cv::Mat& abc,cv::Mat& n, cv::Mat& coordinates, bool& step);
	void approximateLine(cv::Mat& depth_image, PointCloudInConstPtr pointcloud, cv::Point2f dotIni, cv::Point2f dotEnd, cv::Mat& abc);
	void scalarProduct(cv::Mat& abc1,cv::Mat& abc2,float& scalarProduct, int& concaveConvex, bool& step);
	void approximateLineFullAndHalfDist (cv::Mat& depth_image, PointCloudInConstPtr pointcloud, cv::Point2f dotIni, cv::Point2f dotEnd, cv::Mat& abc);

	void thinEdges(cv::Mat& edgePicture, int xy);
	void drawLines(cv::Mat& plotXY, cv::Mat& coordinates, cv::Mat& abc);
	void drawLineAlongN(cv::Mat& plotZW, cv::Mat& coordinates, cv::Mat& n);

	void deriv2nd3pts (cv::Mat threePoints, float& deriv);
	void deriv2nd5pts (cv::Mat threePoints, float& deriv);
	void deriv2nd (cv::Mat depth_image,PointCloudInConstPtr cloud, cv::Point2f dotStart, cv::Point2f dotStop, float& deriv);



	float stepThreshold_;	//minimum distance which is detected as a step in depth coordinates
	float offsetConcConv_;	//how much the gradients need to differ
	int lineLength_;	//depth coordinates along two lines with length lineLength/2 are considered
	int windowX_;	//size of visualization windows in x-direction
	int windowY_;
	float th_plane_;	//threshold of scalarproduct. Only smaller values are taken into account for edges.
	float th_edge_;		//threshold of scalarproduct. Only larger values are taken into account for edges. Should be negative.
};


#include "cob_surface_classification/impl/edge_detection.hpp"


#endif /* EDGE_DETECTION_H_ */
