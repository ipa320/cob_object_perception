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

#include <sys/time.h>

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
	    	edge.primitive_size(&image, &results);
	    	count++;
	    	std::cout<< "Feature computing completed: "<<(count/number_of_images)*100<<"%   Picnum"<<count<<std::endl;

	    	if(results.colorfulness!=results.colorfulness)std::cout<<"colorfulness failure"<<std::endl;
	    	if(results.dom_color!=results.dom_color)std::cout<<"domcolor failure"<<std::endl;
	    	if(results.dom_color2!=results.dom_color2)std::cout<<"domcolor2 failure"<<std::endl;


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
	  std::cout<<"fin"<<std::endl;
}

void compute_textures::compute_test_data()
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
	std::vector<std::string> Label;

//		struct feature_results results;
    		struct color_vals color_results;
    		cv::Mat image;


	cv::Mat train_data = cv::Mat::zeros(129,16,CV_32F);

	  if ((pDIR = opendir("/home/rmb-dh/datasetTextur/Test_data/test")))
	  {
	    while ((entry = readdir(pDIR)))
	    {
	      if (entry->d_type == 0x8)//strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
	      {
	        str = "/home/rmb-dh/datasetTextur/Test_data/test/";
	        name = entry->d_name;
	        str.append(entry->d_name);

	    	image = cv::imread(str);
	    	std::cout<<str<<":   ";

	    	std::cout<<entry->d_name<<std::endl;
	    	Label.push_back(entry->d_name);


	    	std::cout<<str<<":   ";

//	    		color_parameter color = color_parameter();
////	    		try{
//	    		color.get_color_parameter(image, &results);
//	    		}catch(...)
//	    		    	{
//	    		    		std::cout<<entry->d_name<<"bild das fehler erzeugt"<<std::endl;
//	    		    	}

	    	texture_features edge = texture_features();

	    	edge.primitive_size(&image, &results);

//	    	edge.~texture_features();

//	    	train_data.at<float>(count,0) = results.colorfulness; // 3: colorfullness
//	    	train_data.at<float>(count,1) = results.dom_color; // 4: dominant color
//	    	train_data.at<float>(count,2) = results.dom_color2; // 5: dominant color2
//	    	train_data.at<float>(count,3) = results.v_mean; //6: v_mean
//	    	train_data.at<float>(count,4) = results.v_std; // 7: v_std
//	    	train_data.at<float>(count,5) = results.s_mean; // 8: s_mean
//	    	train_data.at<float>(count,6) = results.s_std; // 9: s_std
//	    	train_data.at<float>(count,7) = results.avg_size; // 10: average primitive size
//	    	train_data.at<float>(count,8) = results.prim_num; // 11: number of primitives
//	    	train_data.at<float>(count,9) = results.prim_strength; // 12: strength of primitives
//	    	train_data.at<float>(count,10) = results.prim_regularity; // 13: regularity of primitives
//	    	train_data.at<float>(count,11) = results.contrast; // 14: contrast:
//	    	train_data.at<float>(count,12) = results.line_likeness; // 15: line-likeness
////	Nicht implementiert	    	train_data.at<float>(count,13) = results.roughness; // 16: 3D roughness
//	    	train_data.at<float>(count,13) = results.direct_reg; // 17: directionality/regularity
//	    	train_data.at<float>(count,14) = results.lined; // 18: lined
//	    	train_data.at<float>(count,15) = results.checked; // 19: checked
	    	count++;


//std::cout<<std::endl;
	      }
	      inn.close();
	    }
	    closedir(pDIR);
	  }
	  std::cout<<"fin"<<std::endl;

	  //	Save data to train;
//	  	cv::FileStorage fs("/home/rmb-dh/Test_dataset/test_data.yml", cv::FileStorage::WRITE);
//	  	fs << "test_data" << train_data;
//	  //	Save responsvalues of traindata
//	  	cv::FileStorage fsw("/home/rmb-dh/Test_dataset/test_data_label.yml", cv::FileStorage::WRITE);
//	  	fsw << "test_label" << Label;
}

void compute_textures::compute_textures_one()
{

	struct feature_results results_modul, results_whole;


	        std::string str = "/home/rmb-dh/test_dataset/Bread_06_02.JPG";


	    	cv::Mat image = cv::imread(str);
	    	std::cout<<str<<":   ";

//	    		struct color_vals color_results;
//	    		color_parameter color = color_parameter();
//	    		color.get_color_parameter(image, &results_whole);
//
	    	struct timeval zeit;
	    	gettimeofday(&zeit, 0);
//
	    	texture_features edge = texture_features();
	    	edge.primitive_size(&image, &results_whole);
	    	struct timeval zeit2;
	    	gettimeofday(&zeit2, 0);


	    	edge.compute_features(&image, &results_modul);
	    	struct timeval zeit3;
	    	gettimeofday(&zeit3, 0);
//	    	std::cout<< "Feature computing completed: "<<(count/number_of_images)*100<<"%   Picnum"<<count<<std::endl;

	    	 std::cout << zeit3.tv_sec << ':' << zeit3.tv_usec << std::endl;
	    	 std::cout << zeit2.tv_sec << ':' << zeit2.tv_usec << std::endl;
	    	 std::cout << zeit.tv_sec << ':' << zeit.tv_usec << std::endl;
//	    	 std::cout << zeit3.tv_sec-zeit2.tv_sec << ':' << zeit3.tv_usec-zeit2.tv_usec << std::endl;
//	    	 std::cout << zeit2.tv_sec-zeit.tv_sec << ':' << zeit2.tv_usec-zeit.tv_sec <<"whole"<< std::endl;
//	     	 std::cout << zeit3.tv_sec-zeit2.tv_sec << ':' << zeit3.tv_usec-zeit2.tv_usec<<"module " << std::endl;


	    	std::cout<<"old results"<<std::endl;
	    	std::cout<<"3: colorfullness:              "<< results_whole.colorfulness<<std::endl;
	    	std::cout<<"4: dominant color:             "<<results_whole.dom_color<<std::endl;
	    	std::cout<<"5: dominant color2:            "<<results_whole.dom_color2<<std::endl;
	    	std::cout<<"6: v_mean:                     "<<results_whole.v_mean<<std::endl;
	    	std::cout<<"7: v_std:                      "<<results_whole.v_std<<std::endl;
	    	std::cout<<"8: s_mean:                     "<<results_whole.s_mean<<std::endl;
	    	std::cout<<"9: s_std:                      "<<results_whole.s_std<<std::endl;
	    	std::cout<<"10: average primitive size:     "<<results_whole.avg_size<<std::endl;
	    	std::cout<<"11: number of primitives:       "<<results_whole.prim_num<<std::endl;
	    	std::cout<<"12: strength of primitives:    "<<results_whole.prim_strength<<std::endl;
	    	std::cout<<"13: regularity of primitives:  "<<results_whole.prim_regularity<<std::endl;
	    	std::cout<<"14: contrast:                  "<<results_whole.contrast<<std::endl;
	    	std::cout<<"15: line-likeness:             "<<results_whole.line_likeness<<std::endl;
	    	std::cout<<"16: 3D roughness:              "<<results_whole.roughness<<std::endl;
	    	std::cout<<"17: directionality/regularity: "<<results_whole.direct_reg<<std::endl;
	    	std::cout<<"18: lined:                     "<<results_whole.lined<<std::endl;
	    	std::cout<<"20: checked:                   "<<results_whole.checked<<std::endl;
	    	std::cout<<std::endl<<"new results"<<std::endl;
	    	std::cout<<"3: colorfullness:              "<< results_modul.colorfulness<<std::endl;
	    	std::cout<<"4: dominant color:             "<<results_modul.dom_color<<std::endl;
	    	std::cout<<"5: dominant color2:            "<<results_modul.dom_color2<<std::endl;
	    	std::cout<<"6: v_mean:                     "<<results_modul.v_mean<<std::endl;
	    	std::cout<<"7: v_std:                      "<<results_modul.v_std<<std::endl;
	    	std::cout<<"8: s_mean:                     "<<results_modul.s_mean<<std::endl;
	    	std::cout<<"9: s_std:                      "<<results_modul.s_std<<std::endl;
	    	std::cout<<"10: average primitive size:     "<<results_modul.avg_size<<std::endl;
	    	std::cout<<"11: number of primitives:       "<<results_modul.prim_num<<std::endl;
	    	std::cout<<"12: strength of primitives:    "<<results_modul.prim_strength<<std::endl;
	    	std::cout<<"13: regularity of primitives:  "<<results_modul.prim_regularity<<std::endl;
	    	std::cout<<"14: contrast:                  "<<results_modul.contrast<<std::endl;
	    	std::cout<<"15: line-likeness:             "<<results_modul.line_likeness<<std::endl;
	    	std::cout<<"16: 3D roughness:              "<<results_modul.roughness<<std::endl;
	    	std::cout<<"17: directionality/regularity: "<<results_modul.direct_reg<<std::endl;
	    	std::cout<<"18: lined:                     "<<results_modul.lined<<std::endl;
	    	std::cout<<"20: checked:                   "<<results_modul.checked<<std::endl;
	    	std::cout<<std::endl;
	      	std::cout<<std::endl;


}
