#include "create_train_data.h"
#include <cob_texture_categorization/texture_categorization.h>

#include "compute_textures.h"
#include "create_lbp.h"
#include "splitandmerge.h"
#include "texture_features.h"
#include "write_xml.h"
#include "color_parameter.h"
#include <cv.h>
#include <highgui.h>


#include <iostream>
#include <fstream>
#include <dirent.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/time.h>


create_train_data::create_train_data()
{
}

void create_train_data::compute_data()
{

	std::vector<std::string> texture_class;
	texture_class.push_back("Alufoil");				//0
	texture_class.push_back("Asphalt");
	texture_class.push_back("Bookshelf");
	texture_class.push_back("Bread");
	texture_class.push_back("Brick");
	texture_class.push_back("Broccoli");
	texture_class.push_back("Carpet");
	texture_class.push_back("Cauliflower");
	texture_class.push_back("CD");
	texture_class.push_back("Chocolate");
	texture_class.push_back("Clock");				//10
	texture_class.push_back("Coconut");
	texture_class.push_back("Coffee");
	texture_class.push_back("Concrete");
	texture_class.push_back("Corduroy");
	texture_class.push_back("Cork");
	texture_class.push_back("Cotton");
	texture_class.push_back("Cracker");
	texture_class.push_back("Cup");
	texture_class.push_back("Flakes");
	texture_class.push_back("Flour");				//20
	texture_class.push_back("Foam");
	texture_class.push_back("Football");
	texture_class.push_back("Fork");
	texture_class.push_back("Fur");
	texture_class.push_back("Granite");
	texture_class.push_back("Grapes");
	texture_class.push_back("Ingrain");
	texture_class.push_back("Jalousie");
	texture_class.push_back("Kiwi");
	texture_class.push_back("Knife");				//30
	texture_class.push_back("Leather");
	texture_class.push_back("Lemon");
	texture_class.push_back("Lime");
	texture_class.push_back("Linen");
	texture_class.push_back("Marble");
	texture_class.push_back("Mouse");
	texture_class.push_back("Orange");
	texture_class.push_back("Parsley");
	texture_class.push_back("Pasta");
	texture_class.push_back("Pavingstone");			//40
	texture_class.push_back("PCKeyboard");
	texture_class.push_back("Pineapple");
	texture_class.push_back("Plate");
	texture_class.push_back("Rice");
	texture_class.push_back("Sand");
	texture_class.push_back("Smarties");
	texture_class.push_back("Sponge");
	texture_class.push_back("Spoon");
	texture_class.push_back("Styrofoam");
	texture_class.push_back("Telephone");			//50
	texture_class.push_back("Texwallpaper");
	texture_class.push_back("Tiles");
	texture_class.push_back("Tomato");
	texture_class.push_back("Varnished");
	texture_class.push_back("Washingmachine");
	texture_class.push_back("Wood");





//	texture_class.push_back("Exit");

	cv::Mat train_data = cv::Mat::zeros(133, 16, CV_32FC1);
	cv::Mat responses = cv::Mat::zeros(133, 1, CV_32FC1);


		std::string str, name;
		DIR *pDIR;
		struct dirent *entry;
		unsigned x, y, width, height;
		std::string word;

		double number_of_images = 1224;
		double count=0;

std::cout<<"BEGINN";
std::vector<int> fehler;
	for(int j=0;j<texture_class.size();j++)
	{




	std::string path = "/home/rmb-dh/datasetTextur/Test_data/test/";
	path = path + texture_class[j];
	const char *p;
	p=path.c_str();



		  if ((pDIR = opendir(p)))
		  {

		    while ((entry = readdir(pDIR)))
		    {
		      //if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0 )
		      if(entry->d_type == 0x8)		//File: 0x8, Folder: 0x4
		    {
		        str = path + "/";
		        name = entry->d_name;
		        str.append(entry->d_name);

		    	cv::Mat image = cv::imread(str);
		    	std::cout<<str<<":   ";
		    		struct feature_results results;
		    		struct color_vals color_results;
		    		color_parameter color = color_parameter();
		    		color.get_color_parameter(image, &results);


		    	texture_features edge = texture_features();
		    	edge.primitive_size(&image, &results);

		    	responses.at<float>(count,0)=j;
		    	train_data.at<float>(count,0) = results.colorfulness; // 3: colorfullness
		    	train_data.at<float>(count,1) = results.dom_color; // 4: dominant color
		    	train_data.at<float>(count,2) = results.dom_color2; // 5: dominant color2
		    	train_data.at<float>(count,3) = results.v_mean; //6: v_mean
		    	train_data.at<float>(count,4) = results.v_std; // 7: v_std
		    	train_data.at<float>(count,5) = results.s_mean; // 8: s_mean
		    	train_data.at<float>(count,6) = results.s_std; // 9: s_std
		    	train_data.at<float>(count,7) = results.avg_size; // 10: average primitive size
		    	train_data.at<float>(count,8) = results.prim_num; // 11: number of primitives
		    	train_data.at<float>(count,9) = results.prim_strength; // 12: strength of primitives
		    	train_data.at<float>(count,10) = results.prim_regularity; // 13: regularity of primitives
		    	train_data.at<float>(count,11) = results.contrast; // 14: contrast:
		    	train_data.at<float>(count,12) = results.line_likeness; // 15: line-likeness
//	Nicht implementiert	    	train_data.at<float>(count,13) = results.roughness; // 16: 3D roughness
		    	train_data.at<float>(count,13) = results.direct_reg; // 17: directionality/regularity
		    	train_data.at<float>(count,14) = results.lined; // 18: lined
		    	train_data.at<float>(count,15) = results.checked; // 19: checked
		    	for(int i=0;i<16;i++)
		    	{
		    		if(train_data.at<float>(count,i)!=train_data.at<float>(count,i))
		    		{
		    			fehler.push_back(i);
		    			train_data.at<float>(count,i)=0;
		    		}
		    	}
				count++;
		    	std::cout<< "Feature computing completed: "<<(count/number_of_images)*100<<"%   Picnum"<<count<<std::endl;

		      }
		    }
		    closedir(pDIR);
		  }
	}

 	for(int i=0;i<fehler.size();i++)
		    	{
 						std::cout<<"fehler in"<<fehler[i]<<std::endl;
		    	}

////	Save data to train;
//	cv::FileStorage fs("/home/rmb-dh/Test_dataset/training_data.yml", cv::FileStorage::WRITE);
//	fs << "Training_data" << train_data;
////	Save responsvalues of traindata
//	cv::FileStorage fsw("/home/rmb-dh/Test_dataset/train_data_respons.yml", cv::FileStorage::WRITE);
//	fsw << "Training_label" << responses;

	//	Save data to test;
		cv::FileStorage fs("/home/rmb-dh/Test_dataset/test_data.yml", cv::FileStorage::WRITE);
		fs << "test_data" << train_data;
	//	Save responsvalues of test
		cv::FileStorage fsw("/home/rmb-dh/Test_dataset/test_data_label.yml", cv::FileStorage::WRITE);
		fsw << "test_label" << responses;

	std::cout<<"Training completed"<<std::endl;
}


