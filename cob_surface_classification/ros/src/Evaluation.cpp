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
	//check all pixels in imOrigin according to their color: how many of them have been colored as in imComp?


	//dont't consider border of image because of inaccurate computations due to cut neighbourhood.
	for (unsigned int v=10; v<imOrigin.rows-10; v++)
	{
		for (unsigned int u=10; u<imOrigin.cols-10; u++)
		{

			c.countCompared++;
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
				c.countEdge++;
			}
			//other colors
			else
			{
				if(std::abs((int)(hsv_gt.h - HUE_GREEN)) < HUE_DIFF_TH)			c.countPlane++;
				else if (std::abs((int)(hsv_gt.h - HUE_YELLOW)) < HUE_DIFF_TH) 	c.countConc++;
				else if (std::abs((int)(hsv_gt.h - HUE_MAGENTA)) < HUE_DIFF_TH)	c.countConv++;
			}

			//comparisons
			//both black
			if((hsv_gt.v < 20) && (hsv_test.v < 20))
			{
				c.countCorrect++;
				c.countCorrectEdge++;
			}
			//other colors
			else if(std::abs((int)(hsv_test.h - hsv_gt.h)) < HUE_DIFF_TH)
			{
				c.countCorrect++;
				if(std::abs((int)(hsv_gt.h - HUE_GREEN)) < HUE_DIFF_TH)			c.countCorrectPlane++;
				else if (std::abs((int)(hsv_gt.h - HUE_YELLOW)) < HUE_DIFF_TH) 	c.countCorrectConc++;
				else if (std::abs((int)(hsv_gt.h - HUE_MAGENTA)) < HUE_DIFF_TH)	c.countCorrectConv++;
			}
		}
	}
}

int Evaluation::compareClassification(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr gt, cv::Mat gt_color_image)
{
	//compare all points in clusters of type edge,plane,concave,convex to the colors of the ground truth


	struct count c_r = {0,0,0,0,0,0,0,0,0,0,0};
	struct count c_p = {0,0,0,0,0,0,0,0,0,0,0};

	//points not taken into account until the end (no class assigned) remain white
	cv::Mat test_image = cv::Mat::ones(gt->height, gt->width, CV_8UC3);
	clusterTypesToColorImage(test_image, gt->height, gt->width);

/*
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


	//comparison using the clusters
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
	compareImagesUsingColor(test_image, gt_color_image, c_p);

	struct percentages p_p = {0,0,0,0};
	if (c_p.countConc == 0) p_p.conc = 0;  else p_p.conc  = (c_p.countCorrectConc  / c_p.countConc)  * 100;
	if (c_p.countConv == 0) p_p.conv = 0;  else p_p.conv  = (c_p.countCorrectConv  / c_p.countConv)  * 100;
	if (c_p.countEdge == 0) p_p.edge = 0;  else p_p.edge  = (c_p.countCorrectEdge  / c_p.countEdge)  * 100;
	if (c_p.countPlane == 0) p_p.plane = 0;  else p_p.plane = (c_p.countCorrectPlane / c_p.countPlane) * 100;

	std::cout <<"Precision:\n ------------------------\n";
	std::cout <<  "correctly classified points: " << c_p.countCorrect << " out of: "<< c_p.countCompared << " compared points\n";
	std::cout << "Overall number of points in cloud: " << gt->size() << std::endl;
	std::cout << "correctly classified points of type\n -plane:   \t" << c_p.countCorrectPlane <<" out of " <<c_p.countPlane <<"\n -concave:\t" << c_p.countCorrectConc <<" out of " <<c_p.countConc <<"\n -convex:\t" << c_p.countCorrectConv <<" out of " <<c_p.countConv <<"\n -edge: \t" << c_p.countCorrectEdge <<" out of " <<c_p.countEdge << std::endl;

	std::cout <<"Concave " << p_p.conc << " %\nConvex " << p_p.conv << " %\nEdge "<< p_p.edge << " %\nPlane " << p_p.plane << " %\n";

	//recall
	//----------------------------------------------------------
	compareImagesUsingColor(gt_color_image, test_image, c_r);

	struct percentages p_r = {0,0,0,0};
	if (c_r.countConc == 0) p_r.conc = 0;  else p_r.conc  = (c_r.countCorrectConc  / c_r.countConc)  * 100;
	if (c_r.countConv == 0) p_r.conv = 0;  else p_r.conv  = (c_r.countCorrectConv  / c_r.countConv)  * 100;
	if (c_r.countEdge == 0) p_r.edge = 0;  else p_r.edge  = (c_r.countCorrectEdge  / c_r.countEdge)  * 100;
	if (c_r.countPlane == 0) p_r.plane = 0;  else p_r.plane = (c_r.countCorrectPlane / c_r.countPlane) * 100;

	std::cout <<"Recall:\n ------------------------\n";
	std::cout <<  "correctly classified points: " << c_r.countCorrect << " out of: "<< c_r.countCompared << " compared points\n";
	std::cout << "Overall number of points in cloud: " << gt->size() << std::endl;
	std::cout << "correctly classified points of type\n -plane:   \t" << c_r.countCorrectPlane <<" out of " <<c_r.countPlane <<"\n -concave:\t" << c_r.countCorrectConc <<" out of " <<c_r.countConc <<"\n -convex:\t" << c_r.countCorrectConv <<" out of " <<c_r.countConv <<"\n -edge: \t" << c_r.countCorrectEdge <<" out of " <<c_r.countEdge << std::endl;

	std::cout <<"Concave " << p_r.conc << " %\nConvex " << p_r.conv << " %\nEdge "<< p_r.edge << " %\nPlane " << p_r.plane << " %\n";

	return 0;
}
