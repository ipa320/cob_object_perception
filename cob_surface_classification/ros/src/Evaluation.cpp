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

int Evaluation::compareClassification(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr gt, cv::Mat gt_color_image)
{
	//compare all points in clusters of type edge,plane,concave,convex to the colors of the ground truth

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

	cv::Mat test_image = cv::Mat::zeros(gt->height, gt->width, CV_8UC3);


	//comparison
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
				if((hsv_test.h - hsv_gt.h) < 3)	//same color (independent from light conditions)
				{
					countCorrect++;
					switch(c_it->type)
					{
						case I_EDGE: countCorrectEdge++; break;
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
	std::cout <<"Precision:\n ------------------------\n";
	std::cout << "compared points: "<< countCompared << ", correctly classified points: " << countCorrect << "\n";
	std::cout << "Overall number of points in cloud: " << gt->size() << ", number of points not taken into account: " << countNoColorAssigned<<std::endl;
	std::cout << "correctly classified points of type\n -plane:   \t" << countCorrectPlane <<" out of " <<countPlane <<"\n -concave:\t" << countCorrectConc <<" out of " <<countConc <<"\n -convex:\t" << countCorrectConv <<" out of " <<countConv <<"\n -edge: \t" << countCorrectEdge <<" out of " <<countEdge << std::endl;

	cv::imshow("classification",test_image);
	cv::waitKey(30);

	//recall evaluation
	//----------------------------------
	int countCorrect_recall = 0;
	int countCorrectEdge_recall = 0;
	int countCorrectPlane_recall = 0;
	int countCorrectConc_recall = 0;
	int countCorrectConv_recall = 0;
	int countCompared_recall = 0;
	int countNoColorAssigned_recall = 0;
	int countEdge_recall = 0;
	int countPlane_recall = 0;
	int countConc_recall = 0;
	int countConv_recall = 0;


	//compare images
	for (unsigned int v=0; v<test_image.rows; v++)
	{
		for (unsigned int u=0; u<test_image.cols; u++)
		{

			pcl::PointXYZHSV hsv_gt;	//Point of ground truth
			pcl::PointXYZHSV hsv_test;	//Point of classified cloud
			pcl::PointXYZRGB rgb_gt;
			pcl::PointXYZRGB rgb_test;
			rgb_gt.b = gt_color_image.at<cv::Vec3b>(v,u)[0];
			rgb_gt.g = gt_color_image.at<cv::Vec3b>(v,u)[1];
			rgb_gt.r = gt_color_image.at<cv::Vec3b>(v,u)[2];
			rgb_test.b = test_image.at<cv::Vec3b>(v,u)[0];
			rgb_test.g = test_image.at<cv::Vec3b>(v,u)[1];
			rgb_test.r = test_image.at<cv::Vec3b>(v,u)[2];

			//check for black first (no determining hue value)
			//if(rgb_gt.r < 40 && rgb_gt.g < 5 && rgb_gt.b < 40 && rgb_test.r < 40 && rgb_test.g < 5 && rgb_test.b < 5)
//			//{
//				countCorrectEdge++;
//				countCorrect_recall++;
//			}
			//check for other colors
//			else
//			{
				pcl::PointXYZRGBtoXYZHSV ( 	rgb_test, hsv_test);
				pcl::PointXYZRGBtoXYZHSV ( 	rgb_gt, hsv_gt);
				if((hsv_test.h - hsv_gt.h) <3)
				{

					//black
					if((hsv_gt.v < 20) && (hsv_test.v < 20))
					{
						countCorrect_recall++;
						countCorrectEdge++;
					}
					//other colors
					else
					{
					switch((int)hsv_gt.h)
					{
					case HUE_GREEN: countCorrectPlane++; break;
					case HUE_YELLOW: countCorrectConc++; break;
					case HUE_MAGENTA: countCorrectConv++; break;
					}
					}

				}
			//}

		}
	}

	std::cout << "recall:\n-------------------------------------\n";
	std::cout << "correctly classified points: " << countCorrect_recall;

	return 0;
}
