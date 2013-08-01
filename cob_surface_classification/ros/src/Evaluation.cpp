/*
 * Evaluation.cpp
 *
 *  Created on: Jul 10, 2013
 *      Author: rmb-ce
 */

#include "Evaluation.h"



Evaluation::Evaluation() {
	search_directory = std::string(getenv("HOME")) + "/records/";

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
	int countCorrect = 0;
	int countCompared = 0;

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
				if(gt->points[*idx].rgb == *reinterpret_cast<float*>(&rgb))
					countCorrect++;
			}
		}
	}
	std::cout << "compared points: "<< countCompared << ", correctly classified points: " << countCorrect << "\n";
	return 0;
}
