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


void SurfaceClassification::testFunction(cv::Mat& color_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, cv::Mat& depth_image)
{
	//&: Adresse der Variable
	// -> pointCloud selbst wird verändert! ohne & wird eine Kopie von pointcloud übergeben und nur die kopie verändert
	//bei Funktionsaufruf einfach das objekt übergeben, bei der Deklaration der Funktion ein & an den Datentyp -> die Funktion verwendet eine Referenz/einen Zeiger auf das Objekt!
	//<> template (generisch); XYZRGB: Tiefendaten + Farbbild
	std::cout << "function call \n";

	std::vector<int> test;
	test.size();

	// eigener code
	cv::imshow("image", color_image);
	cv::waitKey(10);



	//computeFPFH(pointcloud);

	//--------------------------------------------------------------------
	//computations with depth_image
	//----------------------------------------------------------------------

	//smooth image
	cv::Mat depth_image_smooth;
	cv::GaussianBlur( depth_image, depth_image_smooth, cv::Size(5,5), 0, 0);

	//first derivatives with Sobel operator
	//-----------------------------------------------------------------------
	cv::Mat x_deriv;
	cv::Mat y_deriv;

	int scale = 1;
	int delta = 0;
	int ddepth = CV_32F;//CV_16S; //depth of output data
	//x-derivative:
	cv::Sobel(depth_image_smooth,x_deriv,ddepth,2,0,3,scale,delta);	//order of derivative in x-direction: 2, in y-direction: 0

	//y-derivative
	cv::Sobel(depth_image_smooth,y_deriv,ddepth,0,2,3,scale,delta);	//order of derivative in y-direction: 2, in x-direction: 0


	//compute gradient magnitude
	cv::Mat grad;
	cv::magnitude(x_deriv,y_deriv,grad);


	//visualization
/*	cv::imshow("depth_image", depth_image);
	cv::waitKey(10);*/

	cv::imshow("depth image smoothed", depth_image_smooth);
	cv::waitKey(10);

/*	cv::imshow("x gradient",x_deriv);
	cv::waitKey(10);

	cv::imshow("y gradient",y_deriv);
	cv::waitKey(10);*/

	cv::imshow("depth image gradient", grad);
	cv::waitKey(10);

	//second derivatives with Laplace operator
	//------------------------------------------------------------------------
	cv::Mat deriv_2;
	int kernel_size = 3;

	cv::Laplacian( depth_image_smooth, deriv_2, ddepth, kernel_size, scale, delta );

	cv::imshow("second derivative", deriv_2);
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
