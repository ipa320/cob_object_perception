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

#include <cob_surface_classification/surface_classification.h>

cv::Mat grad;


static void onMouse( int event, int x,int y,int,void*)
{
	cv::imwrite("Sobelimage.png", grad);
}


void SurfaceClassification::testFunction(cv::Mat& color_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, cv::Mat& depth_image)
{
	//&: Adresse der Variable
	// -> pointCloud selbst wird verändert! ohne & wird eine Kopie von pointcloud übergeben und nur die kopie verändert
	//bei Funktionsaufruf einfach das objekt übergeben, bei der Deklaration der Funktion ein & an den Datentyp -> die Funktion verwendet eine Referenz/einen Zeiger auf das Objekt!
	//<> template (generisch); XYZRGB: Tiefendaten + Farbbild
	std::cout << "function call \n";

	std::vector<int> test;
	test.size();







	//--------------------------------------------------------------------
	//computations with depth_image
	//----------------------------------------------------------------------

	//compute Sobel and Laplacian
	//derivatives(color_image,depth_image);

	//draw two perpendicular lines into the image, and approximate the depth along these lines by linear regression.
	depth_along_lines(color_image, depth_image);


	//visualization
	cv::imshow("image", color_image);
	cv::waitKey(10);

	cv::imshow("depth_image", depth_image);
	cv::waitKey(10);

}

void SurfaceClassification::approximateLine(cv::Mat& depth_image, cv::Mat& plotZW, cv::Point2f dotLeft, cv::Point2f dotRight, int side, cv::Mat& abc)
{
	/*linear regression via least squares minimisation (-> SVD)
	 * ----------------------------------------------------------*/


	int lineLength = dotRight.x - dotLeft.x;
	int windowX = plotZW.cols;
	int windowY = plotZW.rows;

	//write depth of points along the line into a matrix
	//format of one line: [coordinate along the line, depth coordinate, 1]
	cv::Mat coordinates = cv::Mat::zeros(lineLength,3,CV_32FC1);
	cv::LineIterator xIter (depth_image,dotLeft,dotRight);


	for(int v=0; v<xIter.count; v++, ++xIter)
	{
		//anstatt dem Index in x-Richtung die Koordinate in x-Richtung nehmen!!!!
		coordinates.at<float>(v,0) = xIter.pos().x;	//coordinate along the line
		//ACHTUNG: Matrix depth_image: zuerst Zeilenindex (y), dann Spaltenindex (x) !!!!
		coordinates.at<float>(v,1) = depth_image.at<float>(xIter.pos().y, xIter.pos().x);	//depth coordinate
		coordinates.at<float>(v,2) = 1.0;
	}

	//std::cout << "coordinates: \n" << coordinates << "\n";

	cv::Mat sv;	//singular values
	cv::Mat u;	//left singular vectors
	cv::Mat vt;	//right singular vectors, transposed, 3x3
	cv::SVD::compute(coordinates,sv,u,vt);

	//std::cout << "SVD: \n" << vt << "\n";

	//last column of v = last row of vt is x, so that y is minimal
	//cv::Mat abc; //parameters of the approximated line: aw+bz+1 = 0
	abc =  vt.row(2);




	/* draw computed coordinates and approximates lines in plotZW
	 * ------------------------------------------------------------*/
	//projects coordinates on pixel range of window

	cv::Mat wCoordNorm,zCoord;
	//pixel range in window
	int leftBoundary = 0;
	int rightBoundary = 0;
	//scale and shift x-values for visualization:
	if(side == 0)
	{
		//left side
		leftBoundary = 0;
		rightBoundary = windowX /2;
	}
	else if (side == 1)
	{
		//right side
		leftBoundary = windowX/2;
		rightBoundary = windowX;
	}

	cv::normalize(coordinates.col(0),wCoordNorm,leftBoundary,rightBoundary,cv::NORM_MINMAX);
	//std::cout << "wCoordNorm: \n" << wCoordNorm << "\n";


	//compute shift and scale parameters of wCoordNorm
	float min = dotLeft.x; //s.o.
	float max = dotRight.x;
	//float scaleX = (rightBoundary-leftBoundary) /(max-min);


	int scaleDepth = 200;
	zCoord = coordinates.col(1) ;

	//std::cout << "zCoord: \n" << zCoord << "\n";

	for(int v=0; v< coordinates.rows; v++)
	{
		//scale z-value for visualization
		cv::circle(plotZW,cv::Point2f(wCoordNorm.at<float>(v),zCoord.at<float>(v) * scaleDepth),1,CV_RGB(255,255,255),2);
	}

	float x1 = leftBoundary;
	float x2 = rightBoundary;
	float z1 = (-abc.at<float>(2) - abc.at<float>(0) * min) / abc.at<float>(1);
	float z2 = (-abc.at<float>(2) - abc.at<float>(0) * max) / abc.at<float>(1);
	cv::line(plotZW,cv::Point2f(x1 ,z1 *scaleDepth ), cv::Point2f(x2 , z2 * scaleDepth ),CV_RGB(255,255,255),1);
}


void SurfaceClassification::depth_along_lines(cv::Mat& color_image, cv::Mat& depth_image)
{
	int lineLength = 60;
	cv::Point2f dotLeft(color_image.cols/2 -lineLength/2, color_image.rows/2);
	cv::Point2f dotRight(color_image.cols/2 +lineLength/2, color_image.rows/2);
	cv::line(color_image,dotLeft,dotRight,CV_RGB(0,1,0),1);
	cv::Point2f dotUp(color_image.cols/2 , color_image.rows/2 +lineLength/2);
	cv::Point2f dotDown(color_image.cols/2 , color_image.rows/2 -lineLength/2);
	cv::line(color_image,dotUp,dotDown,CV_RGB(0,1,0),1);
	cv::Point2f dotMiddle(color_image.cols/2 , color_image.rows/2 );

	//plot z over w, draw estimated lines
	int windowX = 600;	//size of window in x-direction
	int windowY = 600;

	cv::Mat plotZW (cv::Mat::zeros(windowX,windowY,CV_32FC1));

	cv::Mat abc1 (cv::Mat::zeros(1,3,CV_32FC1));
	approximateLine(depth_image, plotZW,dotLeft,dotMiddle, 0, abc1);
	cv::Mat abc2 (cv::Mat::zeros(1,3,CV_32FC1));
	approximateLine(depth_image, plotZW,dotMiddle,dotRight, 1, abc2);

	//compute scalar product
	float b1 = -(abc1.at<float>(0) + abc1.at<float>(2))/abc1.at<float>(1);
	cv::Mat n1 = (cv::Mat_<float>(1,2) << 1, b1);
	float b2 = -(abc2.at<float>(0) + abc2.at<float>(2))/abc2.at<float>(1);
	cv::Mat n2 = (cv::Mat_<float>(1,2) << 1, b2);

	std::cout << "abc1: " <<abc1 << "\n";
	//abc ist schon normiert, da Ergebnis der SVD eine orthonormale Matrix ist
	cv::Mat scalarProduct = n1 * n2.t();
	std::cout << "scalarProduct: " <<scalarProduct << "\n";

	bool convex = false;
	bool concave = false;
	if(n1.at<float>(1) > 0 && n2.at<float>(1) < 0)
	{
		convex = true;
	}
	else if (n1.at<float>(1) < 0 && n2.at<float>(1) > 0)
	{
		concave = true;
	}


}

void SurfaceClassification::derivatives(cv::Mat& color_image, cv::Mat& depth_image)
{
	//smooth image
	cv::Mat depth_image_smooth = depth_image.clone();
	cv::GaussianBlur( depth_image, depth_image_smooth, cv::Size(13,13), -1, -1);

	//first derivatives with Sobel operator
	//-----------------------------------------------------------------------
	cv::Mat x_deriv;
	cv::Mat y_deriv;

	int scale = 1;
	int delta = 0;
	int ddepth = CV_32F;//CV_16S; //depth (format) of output data
	//x-derivative:
	cv::Sobel(depth_image_smooth,x_deriv,ddepth,1,0,11,scale,delta);	//order of derivative in x-direction: 1, in y-direction: 0

	//y-derivative
	cv::Sobel(depth_image_smooth,y_deriv,ddepth,0,1,11,scale,delta);	//order of derivative in y-direction: 1, in x-direction: 0

	//compute gradient magnitude
	cv::Mat  x_pow2, y_pow2;
	//cv::magnitude(x_deriv,y_deriv,grad);
	cv::pow(x_deriv,2.0, x_pow2);
	cv::pow(y_deriv,2.0, y_pow2);
	cv::sqrt(x_pow2 + y_pow2, grad);


	cv::imshow("depth image smoothed", depth_image_smooth);
	cv::waitKey(10);

	/*	cv::imshow("x gradient",x_deriv);
		cv::waitKey(10);

		cv::imshow("y gradient",y_deriv);
		cv::waitKey(10);*/

	cv::normalize(grad,grad,0,1,cv::NORM_MINMAX);
	cv::setMouseCallback("Sobel",onMouse,0);
	cv::imshow("Sobel", grad);
	cv::waitKey(10);



	//second derivatives with Laplace operator
	//------------------------------------------------------------------------
	cv::Mat deriv_2,temp;
	int kernel_size = 7;

	//cv::Laplacian( depth_image_smooth, temp, ddepth, kernel_size, scale, delta );
	//cv::normalize(deriv_2,deriv_2,-1,2,cv::NORM_MINMAX);

	cv::Mat kernel = (cv::Mat_<float>(5,5) <<  0,          0,          1,          0,          0,
												0,          2,         -8,         2,          0,
												1,         -8,         20,         -8,          1,
												0,          2,         -8,          2,          0,
												0,          0,          1,          0,          0);


	/*cv::Mat kernel = (cv::Mat_< float >(7,7) << 0.1, 0.2, 0.5, 0.8, 0.5, 0.2, 0.1,
			0.2, 1.1, 2.5, 2.7, 2.5, 1.1, 0.2,
			0.5, 2.5, 0, -6.1, 0, 2.5, 0.5,
			0.8, 2.7, -6.1, -20, -6.1, 2.7, 0.8,
			0.5, 2.5, 0, -6.1, 0, 2.5, 0.5,
			0.2, 1.1, 2.5, 2.7, 2.5, 1.1, 0.2,
			0.1, 0.2, 0.5, 0.8, 0.5, 0.2, 0.1);*/
	//LoG (Laplacian and Gaussian) with sigma = 1.4
	/*cv::Mat kernel = (cv::Mat_< float >(9,9) << 0,0,3,2, 2, 2,3,0,0,
													0,2,3,5, 5, 5,3,2,0,
													3,3,5,3, 0, 3,5,3,3,
													2,5,3,-12, -23, -12,3,5,2,
													2,5,0,-23, -40, -23,0,5,2,
													2,5,3,-12, -23, -12,3,5,2,
													3,3,5,3, 0, 3,5,3,3,
													0,2,3,5, 5, 5,3,2,0,
													0,0,3,2, 2, 2,3,0,0);*/


	cv::filter2D(depth_image_smooth, temp, ddepth, kernel);

	float sat = 0.5;

	//Bild immer zeilenweise durchgehen
	for(int v=0; v<temp.rows; v++ )
	{
		for(int u=0; u< temp.cols; u++)
		{
			if(temp.at<float>(v,u) < -sat)
			{
				temp.at<float>(v,u) = - sat;
			}
			else if(temp.at<float>(v,u) > sat)
			{
				temp.at<float>(v,u) =  sat;
			}
		}
	}
	//normalize values to range between 0 and 1, so that all values can be depicted in an image
	cv::normalize(temp,deriv_2,0,1,cv::NORM_MINMAX);

	cv::imshow("Laplacian", deriv_2);
	cv::waitKey(10);
}



void SurfaceClassification::computeFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud)
{
	// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

		//Ptr in constPtr umwandeln
		//pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_cptr = pointcloud;
		//pcl::PCLBase< pcl::PointXYZ >::PointCloudConstPtr cloudWithoutColor;
		// pcl::copyPointCloud(*pointcloud, *cloudWithoutColor);
		pcl::PCLBase< pcl::PointXYZRGB >::PointCloudConstPtr cloud_cptr = pointcloud;
		ne.setInputCloud (cloud_cptr);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).

		pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
		ne.setSearchMethod (tree);

		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

		// Use all neighbors in a sphere of radius 1cm
		ne.setRadiusSearch (0.01);

		// Compute the features
		ne.compute (*cloud_normals);


		// cloud_normals->points.size () should have the same size as the input cloud->points.size ()

		std::cout << "normal estimation finished \n";

		// visualize normals
		pcl::visualization::PCLVisualizer viewer("PCL Viewer");
		viewer.setBackgroundColor (0.0, 0.0, 0);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointcloud);


		viewer.addPointCloud<pcl::PointXYZRGB> (pointcloud, rgb, "sample cloud");
		viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(pointcloud, cloud_normals,10,0.005,"normals");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
		viewer.addCoordinateSystem (1.0);
		viewer.initCameraParameters ();



		//--------------------------------------------------------------------------------
		//compute FPFH

		// Create the FPFH estimation class, and pass the input dataset+normals to it
		//one histogram = concatenation of the four features
		//array of 33 values
		pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
		fpfh.setInputCloud (cloud_cptr);
		fpfh.setInputNormals (cloud_normals);

		// Create an empty kdtree representation, and pass it to the PFH estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr treeFPFH (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());
		fpfh.setSearchMethod (treeFPFH);

		// Output datasets
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

		// Use all neighbors in a sphere of radius 5cm
		// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
		fpfh.setRadiusSearch (0.01);

		// Compute the features
		fpfh.compute (*fpfhs);

		// pfhs->points.size () should have the same size as the input cloud->points.size ()*


		//visualise histogram
		pcl::visualization::PCLHistogramVisualizer histogramViewer;
		histogramViewer.setBackgroundColor(0,0,0);
		histogramViewer.addFeatureHistogram(*fpfhs,33,"FPFH",640,200);


		//for updating, the window has to be closed manually
		while (!viewer.wasStopped ())
		{
			viewer.spinOnce();
			histogramViewer.spinOnce();
		}
		viewer.removePointCloud("sample cloud");
}


