#include "compute_textures.h"
#include "create_lbp.h"
#include "splitandmerge.h"
#include "texture_features.h"
#include "write_xml.h"
#include "color_parameter.h"



#include <iostream>
#include <fstream>
#include <dirent.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

compute_textures::compute_textures()
{
}

void compute_textures::compute_textures_all()
{
	std::ifstream inn;
	std::string str, name;
	DIR *pDIR;
	struct dirent *entry;
	unsigned x, y, width, height;
	std::string word;
	struct feature_results results;
	double number_of_images = 1281;
	double count=0;


	  if ((pDIR = opendir("/home/rmb-dh/test_dataset")))
	  {
	    while ((entry = readdir(pDIR)))
	    {
	      if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
	      {
	        str = "/home/rmb-dh/test_dataset/";
	        name = entry->d_name;
	        str.append(entry->d_name);

	    	cv::Mat image = cv::imread(str);
	    	std::cout<<str<<":   ";

	    		struct color_vals color_results;
	    		color_parameter color = color_parameter();
	    		color.get_color_parameter(image, &results);


	    	texture_features edge = texture_features();
	    	edge.primitive_size(image, &results);
	    	count++;
	    	std::cout<< "Feature computing completed: "<<(count/number_of_images)*100<<"%   Picnum"<<count<<std::endl;

//	    	std::cout<<std::endl;
//	    	std::cout<<"3: colorfullness:              "<< results.colorfulness<<std::endl;
//	    	std::cout<<"4: dominant color:             "<<results.dom_color<<std::endl;
//	    	std::cout<<"5: dominant color2:            "<<results.dom_color2<<std::endl;
//	    	std::cout<<"6: v_mean:                     "<<results.v_mean<<std::endl;
//	    	std::cout<<"7: v_std:                      "<<results.v_std<<std::endl;
//	    	std::cout<<"8: s_mean:                     "<<results.s_mean<<std::endl;
//	    	std::cout<<"9: s_std:                      "<<results.s_std<<std::endl;
//	    	std::cout<<"10: average primitive size:     "<<results.avg_size<<std::endl;
//	    	std::cout<<"11: number of primitives:       "<<results.prim_num<<std::endl;
//	    	std::cout<<"12: strength of primitives:    "<<results.prim_strength<<std::endl;
//	    	std::cout<<"13: regularity of primitives:  "<<results.prim_regularity<<std::endl;
//	    	std::cout<<"14: contrast:                  "<<results.contrast<<std::endl;
//	    	std::cout<<"15: line-likeness:             "<<results.line_likeness<<std::endl;
//	    	std::cout<<"16: 3D roughness:              "<<results.roughness<<std::endl;
//	    	std::cout<<"17: directionality/regularity: "<<results.direct_reg<<std::endl;
//	    	std::cout<<"18: lined:                     "<<results.lined<<std::endl;
//	    	std::cout<<"20: checked:                   "<<results.checked<<std::endl;
//	    	std::cout<<std::endl;
//
//	    	std::cout<<name<<"name";

	    	char string[name.size()];
	    	for(int i=0;i<name.size();i++)
	    	{
	    		string[i]=name[i];
	    	}
	    	char delimiter[] = "_.";
	    	char *ptr;

//	    	 initialisieren und ersten Abschnitt erstellen
	    	ptr = strtok(string, delimiter);
	    	struct data daten;
	    	struct stat status;
	    	std::string patho="/home/rmb-dh/XML/db.xml";
	    	status.create = 1;
	    	daten.data_in[0]=str;
	    	daten.data_in[1]=name;
	    	daten.data_in[4]=ptr;
	    	ptr = strtok(NULL, delimiter);
	    	daten.data_in[3]=ptr;
	    	ptr = strtok(NULL, delimiter);
	    	daten.data_in[2]=ptr;


	    	xml_write write = xml_write();
	    	write.write_data(&status, patho, &daten, &results);

//std::cout<<std::endl;
	      }
	      inn.close();
	    }
	    closedir(pDIR);
	  }
	  std::cin.get();
}
