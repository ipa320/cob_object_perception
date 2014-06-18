#include "cob_texture_categorization/create_train_data.h"
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

#include <vector>
#include <fstream>


create_train_data::create_train_data()
{
}

void create_train_data::compute_data(std::string *path_, int status, std::string *path_save, int number_pictures)
{

	std::vector<std::string> texture_classes;
	texture_classes.push_back("Alufoil");				//0
	texture_classes.push_back("Asphalt");
	texture_classes.push_back("Bookshelf");
	texture_classes.push_back("Bread");
	texture_classes.push_back("Brick");
	texture_classes.push_back("Broccoli");
	texture_classes.push_back("Carpet");
	texture_classes.push_back("Cauliflower");
	texture_classes.push_back("CD");
	texture_classes.push_back("Chocolate");
	texture_classes.push_back("Clock");				//10
	texture_classes.push_back("Coconut");
	texture_classes.push_back("Coffee");
	texture_classes.push_back("Concrete");
	texture_classes.push_back("Corduroy");
	texture_classes.push_back("Cork");
	texture_classes.push_back("Cotton");
	texture_classes.push_back("Cracker");
	texture_classes.push_back("Cup");
	texture_classes.push_back("Flakes");
	texture_classes.push_back("Flour");				//20
	texture_classes.push_back("Foam");
	texture_classes.push_back("Football");
	texture_classes.push_back("Fork");
	texture_classes.push_back("Fur");
	texture_classes.push_back("Granite");
	texture_classes.push_back("Grapes");
	texture_classes.push_back("Ingrain");
	texture_classes.push_back("Jalousie");
	texture_classes.push_back("Kiwi");
	texture_classes.push_back("Knife");				//30
	texture_classes.push_back("Leather");
	texture_classes.push_back("Lemon");
	texture_classes.push_back("Lime");
	texture_classes.push_back("Linen");
	texture_classes.push_back("Marble");
	texture_classes.push_back("Mouse");
	texture_classes.push_back("Orange");
	texture_classes.push_back("Parsley");
	texture_classes.push_back("Pasta");
	texture_classes.push_back("Pavingstone");			//40
	texture_classes.push_back("PCKeyboard");
	texture_classes.push_back("Pineapple");
	texture_classes.push_back("Plate");
	texture_classes.push_back("Rice");
	texture_classes.push_back("Sand");
	texture_classes.push_back("Smarties");
	texture_classes.push_back("Sponge");
	texture_classes.push_back("Spoon");
	texture_classes.push_back("Styrofoam");
	texture_classes.push_back("Telephone");			//50
	texture_classes.push_back("Texwallpaper");
	texture_classes.push_back("Tiles");
	texture_classes.push_back("Tomato");
	texture_classes.push_back("Varnished");
	texture_classes.push_back("Washingmachine");
	texture_classes.push_back("Wood");

	std::vector< std::vector< std::vector< int > > > data_sample_hierarchy(texture_classes.size());			// data_sample_hierarchy[class_index][object_index][sample_index] = entrie_index in feature data matrix

	cv::Mat train_data = cv::Mat::zeros(number_pictures, 16, CV_32FC1);			// matrix of computed features
	cv::Mat responses = cv::Mat::zeros(number_pictures, 1, CV_32FC1);			// matrix of correct classes

	std::string str, name;
	DIR *pDIR;
	struct dirent *entry;
	std::string word;

	double number_of_images = number_pictures;
	double sample_index=0;
	std::string path;

	std::cout<<"BEGIN" << std::endl;
	std::vector<int> errors;
	for(int class_index=0;class_index<(int)texture_classes.size();class_index++)
	{
		path = (*path_) + texture_classes[class_index];
		const char *p;
		p=path.c_str();

		if ((pDIR = opendir(p)))
		{
			while ((entry = readdir(pDIR)))
			{
				//if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0 )
				if (entry->d_type == 0x8) //File: 0x8, Folder: 0x4
				{
					str = path + "/";
					name = entry->d_name;
					str.append(name);

					// create data sample hierarchy
					// determine object number
					int start_pos = name.find("_")+1;
					int end_pos = name.find("_", start_pos);
					std::stringstream object_number_ss;
					object_number_ss << name.substr(start_pos, end_pos-start_pos);
					unsigned int object_number = 0;
					object_number_ss >> object_number;
//					std::cout << "Object number ss: " << object_number_ss.str() << ",  " << object_number << std::endl;
					// determine sample number
					start_pos = end_pos+1;
					end_pos = name.find(".", start_pos);
					std::stringstream sample_number_ss;
					sample_number_ss << name.substr(start_pos, end_pos-start_pos);
					unsigned int sample_number = 0;
					sample_number_ss >> sample_number;
//					std::cout << "Sample number ss: " << sample_number_ss.str() << ",  " << sample_number << std::endl;
					// create entry in hierarchy
					if (data_sample_hierarchy[class_index].size() < object_number)
						data_sample_hierarchy[class_index].resize(object_number);
					if (data_sample_hierarchy[class_index][object_number-1].size() < sample_number)
						data_sample_hierarchy[class_index][object_number-1].resize(sample_number);
					data_sample_hierarchy[class_index][object_number-1][sample_number-1] = sample_index;

					cv::Mat image = cv::imread(str);
					std::cout << str << ":   ";
					feature_results results;
					//struct color_vals color_results;
					color_parameter color = color_parameter(); //Berechnung der Farbfeatures
					color.get_color_parameter(image, &results);

					texture_features edge = texture_features(); //Berechnung der Texturfeatures
					edge.primitive_size(&image, &results);

					responses.at<float>(sample_index, 0) = class_index;
					train_data.at<float>(sample_index, 0) = results.colorfulness; // 3: colorfulness
					train_data.at<float>(sample_index, 1) = results.dom_color; // 4: dominant color
					train_data.at<float>(sample_index, 2) = results.dom_color2; // 5: dominant color2
					train_data.at<float>(sample_index, 3) = results.v_mean; //6: v_mean
					train_data.at<float>(sample_index, 4) = results.v_std; // 7: v_std
					train_data.at<float>(sample_index, 5) = results.s_mean; // 8: s_mean
					train_data.at<float>(sample_index, 6) = results.s_std; // 9: s_std
					train_data.at<float>(sample_index, 7) = results.avg_size; // 10: average primitive size
					train_data.at<float>(sample_index, 8) = results.prim_num; // 11: number of primitives
					train_data.at<float>(sample_index, 9) = results.prim_strength; // 12: strength of primitives
					train_data.at<float>(sample_index, 10) = results.prim_regularity; // 13: regularity of primitives
					train_data.at<float>(sample_index, 11) = results.contrast; // 14: contrast:
					train_data.at<float>(sample_index, 12) = results.line_likeness; // 15: line-likeness
					//	Nicht implementiert	    	train_data.at<float>(count,13) = results.roughness; // 16: 3D roughness
					train_data.at<float>(sample_index, 13) = results.direct_reg; // 17: directionality/regularity
					train_data.at<float>(sample_index, 14) = results.lined; // 18: lined
					train_data.at<float>(sample_index, 15) = results.checked; // 19: checked
					for (int i = 0; i < 16; i++)
					{
						if (train_data.at<float>(sample_index, i) != train_data.at<float>(sample_index, i))
						{
							errors.push_back(i);
							train_data.at<float>(sample_index, i) = 0;
						}
					}
					sample_index++;
					std::cout << "Feature computation completed: " << (sample_index / number_of_images) * 100 << "%   Picnum " << sample_index << std::endl;

				}
			}
			closedir(pDIR);
		}
	}

	for (int i = 0; i < (int)errors.size(); i++)
	{
		std::cout << "Error in feature " << errors[i] << "for some sample." << std::endl;
	}

	std::cout << "Finished reading " << sample_index << " data samples." << std::endl;

	////	Save data to train;
	//	cv::FileStorage fs("/home/rmb-dh/Test_dataset/training_data.yml", cv::FileStorage::WRITE);
	//	fs << "Training_data" << train_data;
	////	Save responsvalues of traindata
	//	cv::FileStorage fsw("/home/rmb-dh/Test_dataset/train_data_respons.yml", cv::FileStorage::WRITE);
	//	fsw << "Training_label" << responses;


	if (status == 0)
	{
		//	Save data;
		std::string data = "database_data.yml";
		std::string path_data = *path_save + data;
		cv::FileStorage fs(path_data, cv::FileStorage::WRITE);
		fs << "database_data" << train_data;

		//	Save response values
		std::string label = "database_data_label.yml";
		std::string path_label = *path_save + label;
		cv::FileStorage fsw(path_label, cv::FileStorage::WRITE);
		fsw << "database_label" << responses;
	}
	else if (status == 1)
	{
		//	Save traindata;
		std::string data = "train_data.yml";
		std::string path_data = *path_save + data;
		//		cv::FileStorage fs("/home/rmb-dh/Test_dataset/test_data.yml", cv::FileStorage::WRITE);
		cv::FileStorage fs(path_data, cv::FileStorage::WRITE);
		fs << "train_data" << train_data;

		//	Save responsvalues
		std::string label = "train_data_label.yml";
		std::string path_label = *path_save + label;
		//		cv::FileStorage fsw("/home/rmb-dh/Test_dataset/test_data_label.yml", cv::FileStorage::WRITE);
		cv::FileStorage fsw(path_label, cv::FileStorage::WRITE);
		fsw << "train_label" << responses;
	}
	else if (status == 2)
	{
		//	Save testdata;
		std::string data = "test_data.yml";
		std::string path_data = *path_save + data;
		//	cv::FileStorage fs("/home/rmb-dh/Test_dataset/test_data.yml", cv::FileStorage::WRITE);
		cv::FileStorage fs(path_data, cv::FileStorage::WRITE);
		fs << "test_data" << train_data;

		//	Save responsvalues
		std::string label = "test_data_label.yml";
		std::string path_label = *path_save + label;
		//		cv::FileStorage fsw("/home/rmb-dh/Test_dataset/test_data_label.yml", cv::FileStorage::WRITE);
		cv::FileStorage fsw(path_label, cv::FileStorage::WRITE);
		fsw << "test_label" << responses;
	}


	// store hierarchy and check validity of hierarchical data structure
	std::string filename = *path_save + "data_hierarchy.txt";
	std::ofstream file(filename.c_str(), std::ios::out);
	if (file.is_open() == true)
	{
		file << texture_classes.size() << std::endl;
		for (unsigned int i=0; i<data_sample_hierarchy.size(); ++i)
		{
			file << texture_classes[i] << "\t" << data_sample_hierarchy[i].size() << std::endl;
//			std::cout << texture_classes[i] << ":" << std::endl;
			if (data_sample_hierarchy[i].size() == 0)
				std::cout << "Warning: class " << texture_classes[i] << " does not contain any objects." << std::endl;
			for (unsigned int j=0; j<data_sample_hierarchy[i].size(); ++j)
			{
				file << "\t" << data_sample_hierarchy[i][j].size() << std::endl;
//				std::cout << "    O" << j+1 << ":" << std::endl;
				if (data_sample_hierarchy[i][j].size() == 0)
					std::cout << "Warning: class " << texture_classes[i] << ", object " << j+1 << " does not contain any samples." << std::endl;
				for (unsigned int k=0; k<data_sample_hierarchy[i][j].size(); ++k)
				{
//					std::cout << "          I" << k+1 << ": " << data_sample_hierarchy[i][j][k] << std::endl;
					file << "\t\t" << data_sample_hierarchy[i][j][k] << std::endl;
				}
			}
		}
	}
	file.close();

	std::cout << "Feature computation on database completed." << std::endl;
}

void create_train_data::load_data_hierarchy(std::string filename, std::vector< std::vector< std::vector< int > > >& data_sample_hierarchy)
{
	std::ifstream file(filename.c_str(), std::ios::in);
	if (file.is_open() == true)
	{
		unsigned int class_number = 0;
		file >> class_number;
		data_sample_hierarchy.resize(class_number);
		for (unsigned int i=0; i<class_number; ++i)
		{
			std::string class_name;
			file >> class_name;
			unsigned int object_number=0;
			file >> object_number;
			data_sample_hierarchy[i].resize(object_number);
			for (unsigned int j=0; j<data_sample_hierarchy[i].size(); ++j)
			{
				unsigned int sample_number=0;
				file >> sample_number;
				data_sample_hierarchy[i][j].resize(sample_number);
				for (unsigned int k=0; k<data_sample_hierarchy[i][j].size(); ++k)
				{
					file >> data_sample_hierarchy[i][j][k];
				}
			}
		}
	}
	file.close();
}
