#include "cob_texture_categorization/create_lbp.h"
#include "cob_texture_categorization/get_mapping.h"
#include "cob_texture_categorization/lbp.h"




#include <iostream>
#include <fstream>
#include <dirent.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>



create_lbp::create_lbp()
{

}


void create_lbp::create_lbp_class(cv::Mat image_in, int radius, int samples, bool ff, double *lbp_hist)
{
	std::string mode = "nh"; //normalized histogram ('nh'), histogram ('h')
	float hist[10];
	std::string mappingtype = "riu2";
	//std::string mappingtype = "nh";
	//hier image verwaltung

	//double lbphist;
	struct mapping mapping;
	//int orbits[samples+2][index-1];
	int index = static_cast<int>(pow(2,samples));
	int table[index];
	for(int i = 0; i<index; i++)
	{
		table[i] = 0;
	}

	// apply specified LBP operation to all imported images
	// LBP with histogram Fourier features
	if(ff == true)
	{

		//get_mapping_hf(samples, &mapping, &orbits, table);
		// int k = 0; nicht benötigt bei einzelnen Images
		//if(image_in.type == 16)
		//{
			//lbphist = construct_hf(*lbp(image_in, radius, samples, &mapping, "nh"), &mapping);
		//}
	}
	else		// rotation invariant uniform LBP
	{
		get_mapping map = get_mapping();
		map.get_mapping_res(samples, mappingtype, &mapping, table);


		//if(image_in.type == 16) //16 == "CV_8U"
		//{
			lbp lbp_hist = lbp();
			lbp_hist.lbp_compute(image_in, radius, samples, &mapping, mode, table, hist);
		//}
	}

	for(int i = 0;i<10;i++)
	{
		lbp_hist[i] =hist[i];
//		std::cout<<hist[i]<<"__";
	}//std::cout<<"createlbp"<<std::endl;



}
