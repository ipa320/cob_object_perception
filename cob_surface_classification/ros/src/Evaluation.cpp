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


void Evaluation::compareImagesUsingColor(cv::Mat imOrigin, cv::Mat imComp)
{
	//check all pixels in imOrigin according to their color: how many of them have been colored as in imComp?

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

	for (unsigned int v=0; v<imOrigin.rows; v++)
	{
		for (unsigned int u=0; u<imOrigin.cols; u++)
		{

			countCompared_recall++;
			pcl::PointXYZHSV hsv_gt;	//Point of ground truth
			pcl::PointXYZHSV hsv_test;	//Point of classified cloud
			pcl::PointXYZRGB rgb_gt;
			pcl::PointXYZRGB rgb_test;
			rgb_gt.b = imOrigin.at<cv::Vec3b>(v,u)[0];
			rgb_gt.g = imOrigin.at<cv::Vec3b>(v,u)[1];
			rgb_gt.r = imOrigin.at<cv::Vec3b>(v,u)[2];
			rgb_test.b = imComp.at<cv::Vec3b>(v,u)[0];
			rgb_test.g = imComp.at<cv::Vec3b>(v,u)[1];
			rgb_test.r = imComp.at<cv::Vec3b>(v,u)[2];


			pcl::PointXYZRGBtoXYZHSV ( 	rgb_test, hsv_test);
			pcl::PointXYZRGBtoXYZHSV ( 	rgb_gt, hsv_gt);




			//black (no determining hue value)
			if(hsv_gt.v < 20)
			{
				countEdge_recall++;
			}
			//other colors
			else
			{


				if(std::abs((int)(hsv_gt.h - HUE_GREEN)) < HUE_DIFF_TH)			countPlane_recall++;
				else if (std::abs((int)(hsv_gt.h - HUE_YELLOW)) < HUE_DIFF_TH) 	countConc_recall++;
				else if (std::abs((int)(hsv_gt.h - HUE_MAGENTA)) < HUE_DIFF_TH)	countConv_recall++;
			}

			//comparisons
			//both black
			if((hsv_gt.v < 20) && (hsv_test.v < 20))
			{
				countCorrect_recall++;
				countCorrectEdge_recall++;
			}
			//other colors
			else if(std::abs((int)(hsv_test.h - hsv_gt.h)) < HUE_DIFF_TH)
			{

				countCorrect_recall++;
				if(std::abs((int)(hsv_gt.h - HUE_GREEN)) < HUE_DIFF_TH)			countCorrectPlane_recall++;
				else if (std::abs((int)(hsv_gt.h - HUE_YELLOW)) < HUE_DIFF_TH) 	countCorrectConc_recall++;
				else if (std::abs((int)(hsv_gt.h - HUE_MAGENTA)) < HUE_DIFF_TH)	countCorrectConv_recall++;
			}
		}

	}


	std::cout << "\nrecall:\n-------------------------------------\n";
	std::cout << "correctly classified points: " << countCorrect_recall<< " out of " << countCompared_recall << " points" <<std::endl;
	std::cout << "correctly classified points of type\n -plane:   \t" << countCorrectPlane_recall <<" out of " <<countPlane_recall <<"\n -concave:\t" << countCorrectConc_recall <<" out of " <<countConc_recall <<"\n -convex:\t" << countCorrectConv_recall <<" out of " <<countConv_recall <<"\n -edge: \t" << countCorrectEdge_recall <<" out of " <<countEdge_recall << std::endl;
	std::cout << "---------------------------------------------------------------\n";
}

int Evaluation::compareClassification(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr gt, cv::Mat gt_color_image)
{
	//compare all points in clusters of type edge,plane,concave,convex to the colors of the ground truth

	struct count
	{
		int countCorrect ;
		int countCorrectEdge;
		int countCorrectPlane ;
		int countCorrectConc ;
		int countCorrectConv ;
		int countCompared ;
		int countNoColorAssigned ;
		int countEdge ;
		int countPlane ;
		int countConc ;
		int countConv ;
	} c_r, c_p;

	c_r = {0,0,0,0,0,0,0,0,0,0,0};

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

	//points not taken into account until the end (no class assigned) remain white
	cv::Mat test_image = cv::Mat::ones(gt->height, gt->width, CV_8UC3);


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
	std::cout <<"Precision:\n ------------------------\n";
	std::cout <<  "correctly classified points: " << countCorrect << " out of: "<< countCompared << " compared points\n";
	std::cout << "Overall number of points in cloud: " << gt->size() << ", number of points other than edge,plane,concave or convex: " << countNoColorAssigned<<std::endl;
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
	std::cout << "neu:---------------------------------------------------\n";
	//precision
	compareImagesUsingColor(test_image, gt_color_image);
	//recall
	compareImagesUsingColor(gt_color_image, test_image);

	std::cout << "so wie bisher:------------------------------------------\n";

	for (unsigned int v=0; v<test_image.rows; v++)
	{
		for (unsigned int u=0; u<test_image.cols; u++)
		{

			countCompared_recall++;
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


			pcl::PointXYZRGBtoXYZHSV ( 	rgb_test, hsv_test);
			pcl::PointXYZRGBtoXYZHSV ( 	rgb_gt, hsv_gt);




			//black (no determining hue value)
			if(hsv_gt.v < 20)
			{
				countEdge_recall++;
				if(hsv_test.v < 20)
				{
					countCorrect_recall++;
					countCorrectEdge_recall++;
				}
			}
			else
			{
				//other colors

				if(std::abs(hsv_gt.h - HUE_GREEN) < HUE_DIFF_TH)			countPlane_recall++;
				else if (std::abs(hsv_gt.h - HUE_YELLOW) < HUE_DIFF_TH) 	countConc_recall++;
				else if (std::abs(hsv_gt.h - HUE_MAGENTA) < HUE_DIFF_TH)	countConv_recall++;


				if(std::abs((int)(hsv_test.h - hsv_gt.h)) < HUE_DIFF_TH)
				{

					countCorrect_recall++;

					if(std::abs(hsv_gt.h - HUE_GREEN) < HUE_DIFF_TH)			countCorrectPlane_recall++;
					else if (std::abs(hsv_gt.h - HUE_YELLOW) < HUE_DIFF_TH) 	countCorrectConc_recall++;
					else if (std::abs(hsv_gt.h - HUE_MAGENTA) < HUE_DIFF_TH)	countCorrectConv_recall++;
				}
			}

		}
	}

	std::cout << "\nrecall:\n-------------------------------------\n";
	std::cout << "correctly classified points: " << countCorrect_recall<< " out of " << countCompared_recall << " points" <<std::endl;
	std::cout << "correctly classified points of type\n -plane:   \t" << countCorrectPlane_recall <<" out of " <<countPlane_recall <<"\n -concave:\t" << countCorrectConc_recall <<" out of " <<countConc_recall <<"\n -convex:\t" << countCorrectConv_recall <<" out of " <<countConv_recall <<"\n -edge: \t" << countCorrectEdge_recall <<" out of " <<countEdge_recall << std::endl;

	return 0;
}
