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

#include "Evaluation.h"


#define HUE_GREEN 		113
#define HUE_MAGENTA 	0
#define HUE_YELLOW 		55
#define HUE_DIFF_TH		10		//threshold for deviation in hue value which is still ignored (minimum distance between the different hue values that are used)




Evaluation::Evaluation() {
	search_directory = std::string(getenv("HOME")) + "/records/";

	//colors of the classification categories in the ground truth
	color_tab.resize(NUM_LABELS);
	color_tab[I_EDGE] = EVAL_COL_EDGE;
	color_tab[I_PLANE] = EVAL_COL_PLANE;
	color_tab[I_CONCAVE]= EVAL_COL_CONC;
	color_tab[I_CONVEX]= EVAL_COL_CONV;
}

Evaluation::~Evaluation() {
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

float Evaluation::divide(float a, float b)
{
	//compute a/b
	if (b == 0)
		if(a == 0)
			return 100;
		else
			return 0;
	else
		return (a  / b)  * 100;
}


void Evaluation::clusterTypesToColorImage(cv::Mat& test_image, unsigned int height,unsigned int width)
{
	ST::CH::ClusterPtr c_it,c_end;
	for ( boost::tie(c_it,c_end) = clusterHandler->getClusters(); c_it != c_end; ++c_it)
	{

		if(c_it->type == I_EDGE || c_it->type == I_PLANE || c_it->type == I_CONCAVE || c_it->type == I_CONVEX )
		{
			uint32_t rgb = color_tab[c_it->type];

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


void Evaluation::compareImagesUsingColor(cv::Mat imOrigin, cv::Mat imComp,  Evaluation::count& c)
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

			pcl::PointXYZRGBtoXYZHSV ( 	rgb_comp, hsv_comp);
			pcl::PointXYZRGBtoXYZHSV ( 	rgb_origin, hsv_origin);

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


	struct count c_r = {0,0,0,0,0,0,0,0,0,0,0};
	struct count c_p = {0,0,0,0,0,0,0,0,0,0,0};

	//points not taken into account until the end remain white (->no class assigned)
	cv::Mat test_image = cv::Mat::ones(gt->height, gt->width, CV_8UC3);
	clusterTypesToColorImage(test_image, gt->height, gt->width);

	rec.saveImage(test_image,"prediction");
	rec.saveImage(gt_color_image,"gt");


/*
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

	 */

	cv::imshow("classification",test_image);
	cv::waitKey(30);



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
