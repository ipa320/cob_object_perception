/*
 * Evaluation.cpp
 *
 *  Created on: Jul 10, 2013
 *      Author: rmb-ce
 */

#include "Evaluation.h"



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

int Evaluation::compareClassification(std::string gt_filename)
{


	pcl::PointCloud<pcl::PointXYZRGB>::Ptr gt (new pcl::PointCloud<pcl::PointXYZRGB>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (search_directory + gt_filename, *gt) == -1)
	{
		PCL_ERROR ("Couldn't read gt-file \n");
		return (-1);
	}

	compareClassification(gt);
}



int Evaluation::compareClassification(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr gt)
{
	//compare all points in clusters of type edge,plane,concave,convex to the colors of the ground truth

	int countCorrect = 0;
	int countCompared = 0;
	int countNoColorAssigned = 0;

	//comparison
	ST::CH::ClusterPtr c_it,c_end;
	for ( boost::tie(c_it,c_end) = clusterHandler->getClusters(); c_it != c_end; ++c_it)
	{
		if(c_it->type == I_EDGE || c_it->type == I_PLANE || c_it->type == I_CONCAVE || c_it->type == I_CONVEX )
		{
			uint32_t rgb = color_tab[c_it->type];
			for (ST::CH::ClusterType::iterator idx=c_it->begin(); idx != c_it->end(); ++idx)
			{
//				std::cout <<"rgb gt: "  << gt->points[*idx].rgb << ", rgb classification: " << *reinterpret_cast<float*>(&rgb) << std::endl;
				countCompared++;

				pcl::PointXYZHSV hsv_gt;
				pcl::PointXYZHSV hsv_test;

				//PUnkt der Klassifizierung
				pcl::PointXYZRGB rgb_test;
				pcl::PointXYZRGB rgb_gt = gt->points[*idx];

				rgb_test.rgb = *reinterpret_cast<float*>(&rgb);
				pcl::PointXYZRGBtoXYZHSV ( 	rgb_test, hsv_test);
				pcl::PointXYZRGBtoXYZHSV ( 	rgb_gt, hsv_gt);
				//if(gt->points[*idx].rgb == *reinterpret_cast<float*>(&rgb))
				if((hsv_test.h - hsv_gt.h) < 2)	//same color (independent from light conditions)
					countCorrect++;
				//std::cout <<"h gt: "  << hsv_gt.h << ", h classification: " << hsv_test.h << std::endl;
			}
		}
		else
		{
			std::cout << "cluster_type: " << c_it->type <<std::endl;
			countNoColorAssigned++;
		}
	}
	std::cout << "compared points: "<< countCompared << ", correctly classified points: " << countCorrect << "\n";
	std::cout << "Punkte der Wolke insgesamt: " << gt->size() << ", nicht berücksichtigte PUnkte: " << countNoColorAssigned<<std::endl;

	return 0;
}
