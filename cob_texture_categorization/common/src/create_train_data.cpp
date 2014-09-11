#include "cob_texture_categorization/create_train_data.h"

#include "cob_texture_categorization/compute_textures.h"
#include "cob_texture_categorization/create_lbp.h"
#include "cob_texture_categorization/splitandmerge.h"
#include "cob_texture_categorization/texture_features.h"
#include "cob_texture_categorization/write_xml.h"
#include "cob_texture_categorization/color_parameter.h"
#include "cob_texture_categorization/ifv_features.h"

#include <highgui.h>


#include <iostream>
#include <fstream>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/time.h>

#include <fstream>


create_train_data::create_train_data()
{
	texture_classes_.push_back("Alufoil");				//0
	texture_classes_.push_back("Asphalt");
	texture_classes_.push_back("Bookshelf");
	texture_classes_.push_back("Bread");
	texture_classes_.push_back("Brick");
	texture_classes_.push_back("Broccoli");
	texture_classes_.push_back("Carpet");
	texture_classes_.push_back("Cauliflower");
	texture_classes_.push_back("CD");
	texture_classes_.push_back("Chocolate");
	texture_classes_.push_back("Clock");				//10
	texture_classes_.push_back("Coconut");
	texture_classes_.push_back("Coffee");
	texture_classes_.push_back("Concrete");
	texture_classes_.push_back("Corduroy");
	texture_classes_.push_back("Cork");
	texture_classes_.push_back("Cotton");
	texture_classes_.push_back("Cracker");
	texture_classes_.push_back("Cup");
	texture_classes_.push_back("Flakes");
	texture_classes_.push_back("Flour");				//20
	texture_classes_.push_back("Foam");
	texture_classes_.push_back("Football");
	texture_classes_.push_back("Fork");
	texture_classes_.push_back("Fur");
	texture_classes_.push_back("Granite");
	texture_classes_.push_back("Grapes");
	texture_classes_.push_back("Ingrain");
	texture_classes_.push_back("Jalousie");
	texture_classes_.push_back("Kiwi");
	texture_classes_.push_back("Knife");				//30
	texture_classes_.push_back("Leather");
	texture_classes_.push_back("Lemon");
	texture_classes_.push_back("Lime");
	texture_classes_.push_back("Linen");
	texture_classes_.push_back("Marble");
	texture_classes_.push_back("Mouse");
	texture_classes_.push_back("Orange");
	texture_classes_.push_back("Parsley");
	texture_classes_.push_back("Pasta");
	texture_classes_.push_back("Pavingstone");			//40
	texture_classes_.push_back("PCKeyboard");
	texture_classes_.push_back("Pineapple");
	texture_classes_.push_back("Plate");
	texture_classes_.push_back("Rice");
	texture_classes_.push_back("Sand");
	texture_classes_.push_back("Smarties");
	texture_classes_.push_back("Sponge");
	texture_classes_.push_back("Spoon");
	texture_classes_.push_back("Styrofoam");
	texture_classes_.push_back("Telephone");			//50
	texture_classes_.push_back("Texwallpaper");
	texture_classes_.push_back("Tiles");
	texture_classes_.push_back("Tomato");
	texture_classes_.push_back("Varnished");
	texture_classes_.push_back("Washingmachine");
	texture_classes_.push_back("Wood");
}

std::vector<std::string> create_train_data::get_texture_classes()
{
	return texture_classes_;
}

void create_train_data::compute_data_handcrafted(std::string path_database_images, std::string path_save, int number_pictures, int mode)
{
	// load labeled ground truth attributes with relation to each image file
	std::map<std::string, std::vector<float> > filenames_gt_attributes;
	std::string path_filenames_gt_attributes = path_save + "ipa_database_filename_attributes.txt";
	load_filenames_gt_attributes(path_filenames_gt_attributes, filenames_gt_attributes);

	create_train_data::DataHierarchyType data_sample_hierarchy(texture_classes_.size());			// data_sample_hierarchy[class_index][object_index][sample_index] = entry_index in feature data matrix

	cv::Mat ground_truth_attribute_matrix = cv::Mat::zeros(number_pictures, 16, CV_32FC1);	// matrix of labeled ground truth attributes
	cv::Mat computed_attribute_matrix = cv::Mat::zeros(number_pictures, 16, CV_32FC1);			// matrix of computed attributes
	cv::Mat class_label_matrix = cv::Mat::zeros(number_pictures, 1, CV_32FC1);			// matrix of correct classes
	cv::Mat base_feature_matrix = cv::Mat::zeros(number_pictures, 22, CV_32FC1); // matrix of computed base features

	std::cout<<"BEGIN" << std::endl;
	double sample_index=0;
	std::stringstream accumulated_error_string;
	std::vector<int> errors;
	for(int class_index=0;class_index<(int)texture_classes_.size();class_index++)
	{
		std::string path = path_database_images + texture_classes_[class_index];
		const char *p;
		p=path.c_str();

		DIR *pDIR;
		struct dirent *entry;
		if ((pDIR = opendir(p)))
		{
			while ((entry = readdir(pDIR)))
			{
				//if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0 )
				if (entry->d_type == 0x8) //File: 0x8, Folder: 0x4
				{
					std::string str = path + "/";
					std::string name = entry->d_name;
					str.append(name);

					if (filenames_gt_attributes.find(name) != filenames_gt_attributes.end())
					{
						for (unsigned int i=0, j=0; i<filenames_gt_attributes[name].size(); ++i)
							if (i!=13)	// one attribute (3d roughness) is currently not implemented here, so just leave it out from gt
							{
								ground_truth_attribute_matrix.at<float>(sample_index, j) = filenames_gt_attributes[name][i];
								++j;
							}
					}
					else
					{
						std::cout << "Error: no entry for file '" << name << "' in filenames_gt_attributes." << std::endl;
						accumulated_error_string << "Error: no entry for file '" << name << "' in filenames_gt_attributes." << std::endl;
					}

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
						data_sample_hierarchy[class_index][object_number-1].resize(sample_number, -1);
					data_sample_hierarchy[class_index][object_number-1][sample_number-1] = sample_index;

					std::cout << str << ":   ";
					cv::Mat image = cv::imread(str);
//					cv::Mat temp;		// hack: downsizing image
//					cv::resize(image, temp, cv::Size(), 0.25, 0.25, cv::INTER_AREA);
//					image = temp;
					feature_results results;
					cv::Mat raw_features = base_feature_matrix.row(sample_index); // todo: check width
					//struct color_vals color_results;
					color_parameter color = color_parameter(); //Berechnung der Farbfeatures
					color.get_color_parameter(image, &results, &raw_features);

					texture_features edge = texture_features(); //Berechnung der Texturfeatures
					edge.primitive_size(&image, &results, &raw_features);

					class_label_matrix.at<float>(sample_index, 0) = class_index;
					computed_attribute_matrix.at<float>(sample_index, 0) = results.colorfulness; // 3: colorfulness
					computed_attribute_matrix.at<float>(sample_index, 1) = results.dom_color; // 4: dominant color
					computed_attribute_matrix.at<float>(sample_index, 2) = results.dom_color2; // 5: dominant color2
					computed_attribute_matrix.at<float>(sample_index, 3) = results.v_mean; //6: v_mean
					computed_attribute_matrix.at<float>(sample_index, 4) = results.v_std; // 7: v_std
					computed_attribute_matrix.at<float>(sample_index, 5) = results.s_mean; // 8: s_mean
					computed_attribute_matrix.at<float>(sample_index, 6) = results.s_std; // 9: s_std
					computed_attribute_matrix.at<float>(sample_index, 7) = results.avg_size; // 10: average primitive size
					computed_attribute_matrix.at<float>(sample_index, 8) = results.prim_num; // 11: number of primitives
					computed_attribute_matrix.at<float>(sample_index, 9) = results.prim_strength; // 12: strength of primitives
					computed_attribute_matrix.at<float>(sample_index, 10) = results.prim_regularity; // 13: regularity of primitives
					computed_attribute_matrix.at<float>(sample_index, 11) = results.contrast; // 14: contrast:
					computed_attribute_matrix.at<float>(sample_index, 12) = results.line_likeness; // 15: line-likeness
					//	not implemented	    	train_data.at<float>(count,13) = results.roughness; // 16: 3D roughness
					computed_attribute_matrix.at<float>(sample_index, 13) = results.direct_reg; // 17: directionality/regularity
					computed_attribute_matrix.at<float>(sample_index, 14) = results.lined; // 18: lined
					computed_attribute_matrix.at<float>(sample_index, 15) = results.checked; // 19: checked
					for (int i = 0; i < 16; i++)
					{
						if (computed_attribute_matrix.at<float>(sample_index, i) != computed_attribute_matrix.at<float>(sample_index, i))
						{
							errors.push_back(i);
							computed_attribute_matrix.at<float>(sample_index, i) = 0;
						}
					}
					sample_index++;
					std::cout << "Feature computation completed: " << (sample_index / number_pictures) * 100 << "%   Picnum " << sample_index << std::endl;

				}
			}
			closedir(pDIR);
		}
	}

	for (int i = 0; i < (int)errors.size(); i++)
	{
		std::cout << "Error in feature " << errors[i] << "for some sample." << std::endl;
	}

	std::cout << "Errors:\n" << accumulated_error_string.str() << std::endl;

	std::cout << "Finished reading " << sample_index << " data samples." << std::endl;

	//	Save computed attributes, class labels and ground truth attributes
	save_texture_database_features(path_save, base_feature_matrix, ground_truth_attribute_matrix, computed_attribute_matrix, class_label_matrix, data_sample_hierarchy, mode);

	std::cout << "Feature computation on database completed." << std::endl;
}


void create_train_data::compute_data_cimpoi(std::string path_database_images, std::string path_save, int number_pictures, int mode, bool generateGMM)
{
	// compute or load GMM
	const int number_gaussian_centers = 256;
	const double image_resize_factor = 0.25;
	IfvFeatures ifv;
	std::string gmm_filename = path_save + "gmm_model.yml";
	if (generateGMM == true)
	{
		// generate a list of all available database image filenames
		std::vector<std::string> image_filenames;
		for(int class_index=0;class_index<(int)texture_classes_.size();class_index++)
		{
			std::string path = path_database_images + texture_classes_[class_index];
			const char *p;
			p=path.c_str();

			DIR *pDIR;
			struct dirent *entry;
			if ((pDIR = opendir(p)))
			{
				while ((entry = readdir(pDIR)))
				{
					if (entry->d_type == 0x8) //File: 0x8, Folder: 0x4
					{
						std::string str = path + "/";
						std::string name = entry->d_name;
						str.append(name);
						image_filenames.push_back(str);
					}
				}
			}
		}
		// compute and store GMM
		ifv.constructGenerativeModel(image_filenames, image_resize_factor, 1000, number_gaussian_centers);
		ifv.saveGenerativeModel(gmm_filename);
	}
	else
		ifv.loadGenerativeModel(gmm_filename);

	// load labeled ground truth attributes with relation to each image file
	std::map<std::string, std::vector<float> > filenames_gt_attributes;
	std::string path_filenames_gt_attributes = path_save + "ipa_database_filename_attributes.txt";
	load_filenames_gt_attributes(path_filenames_gt_attributes, filenames_gt_attributes);

	create_train_data::DataHierarchyType data_sample_hierarchy(texture_classes_.size());			// data_sample_hierarchy[class_index][object_index][sample_index] = entry_index in feature data matrix
	cv::Mat ground_truth_attribute_matrix = cv::Mat::zeros(number_pictures, 17, CV_32FC1);	// matrix of labeled ground truth attributes
	cv::Mat class_label_matrix = cv::Mat::zeros(number_pictures, 1, CV_32FC1);			// matrix of correct classes
	cv::Mat base_feature_matrix = cv::Mat::zeros(number_pictures, 2*128*number_gaussian_centers, CV_32FC1); // matrix of computed base features

	std::cout<<"BEGIN" << std::endl;
	double sample_index=0;
	std::stringstream accumulated_error_string;
	for(int class_index=0;class_index<(int)texture_classes_.size();class_index++)
	{
		std::string path = path_database_images + texture_classes_[class_index];
		const char *p;
		p=path.c_str();

		DIR *pDIR;
		struct dirent *entry;
		if ((pDIR = opendir(p)))
		{
			while ((entry = readdir(pDIR)))
			{
				//if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0 )
				if (entry->d_type == 0x8) //File: 0x8, Folder: 0x4
				{
					std::string str = path + "/";
					std::string name = entry->d_name;
					str.append(name);

					// read out ground truth attributes
					if (filenames_gt_attributes.find(name) != filenames_gt_attributes.end())
					{
						for (unsigned int i=0, j=0; i<filenames_gt_attributes[name].size(); ++i, ++j)
						{
							ground_truth_attribute_matrix.at<float>(sample_index, j) = filenames_gt_attributes[name][i];
						}
					}
					else
					{
						std::cout << "Error: no entry for file '" << name << "' in filenames_gt_attributes." << std::endl;
						accumulated_error_string << "Error: no entry for file '" << name << "' in filenames_gt_attributes." << std::endl;
					}

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
						data_sample_hierarchy[class_index][object_number-1].resize(sample_number, -1);
					data_sample_hierarchy[class_index][object_number-1][sample_number-1] = sample_index;

					// compute IFV base features
					std::cout << str << ":   ";
					cv::Mat base_features = base_feature_matrix.row(sample_index);
					ifv.computeImprovedFisherVector(str, image_resize_factor, number_gaussian_centers, base_features);

					class_label_matrix.at<float>(sample_index, 0) = class_index;

					sample_index++;
					std::cout << "Feature computation completed: " << (sample_index / number_pictures) * 100 << "%   Picnum " << sample_index << std::endl;
				}
			}
			closedir(pDIR);
		}
	}

	std::cout << "Errors:\n" << accumulated_error_string.str() << std::endl;

	std::cout << "Finished reading " << sample_index << " data samples." << std::endl;

	//	Save computed attributes, class labels and ground truth attributes
	save_texture_database_features(path_save, base_feature_matrix, ground_truth_attribute_matrix, cv::Mat(), class_label_matrix, data_sample_hierarchy, mode);

	std::cout << "Feature computation on database completed." << std::endl;
}


void create_train_data::save_texture_database_features(std::string path, const cv::Mat& base_feature_matrix, const cv::Mat& ground_truth_attribute_matrix, const cv::Mat& computed_attribute_matrix, const cv::Mat& class_label_matrix, DataHierarchyType& data_sample_hierarchy, int mode)
{
	//	Save computed attributes, class labels and ground truth attributes
	std::string data = "ipa_database.yml";
	if (mode == 1)
		data = "train_data.yml";
	else if (mode == 2)
		data = "test_data.yml";
	std::string path_data = path + data;
	cv::FileStorage fs(path_data, cv::FileStorage::WRITE);
	fs << "base_feature_matrix" << base_feature_matrix;
	fs << "ground_truth_attribute_matrix" << ground_truth_attribute_matrix;
	fs << "computed_attribute_matrix" << computed_attribute_matrix;
	fs << "class_label_matrix" << class_label_matrix;
	fs.release();

	// store hierarchy and check validity of hierarchical data structure
	std::string filename = path + "ipa_database_hierarchy.txt";
	save_data_hierarchy(filename, data_sample_hierarchy, class_label_matrix.rows);
}

void create_train_data::load_texture_database_features(std::string path, cv::Mat& base_feature_matrix, cv::Mat& ground_truth_attribute_matrix, cv::Mat& computed_attribute_matrix, cv::Mat& class_label_matrix, DataHierarchyType& data_sample_hierarchy)
{
	// load computed attributes, class labels and ground truth attributes
	std::string database_file = path + "ipa_database.yml";
	cv::FileStorage fs(database_file, cv::FileStorage::READ);
	fs["base_feature_matrix"] >> base_feature_matrix;
	fs["ground_truth_attribute_matrix"] >> ground_truth_attribute_matrix;
	fs["computed_attribute_matrix"] >> computed_attribute_matrix;
	fs["class_label_matrix"] >> class_label_matrix;
	fs.release();

	// load class-object-sample hierarchy
	std::string database_hierarchy_file = path + "ipa_database_hierarchy_2fb.txt";
	load_data_hierarchy(database_hierarchy_file, data_sample_hierarchy);

	std::cout << "Texture database features loaded." << std::endl;
}

void create_train_data::save_data_hierarchy(std::string filename, DataHierarchyType& data_sample_hierarchy, int number_samples)
{
	// store hierarchy and check validity of hierarchical data structure
	std::ofstream file(filename.c_str(), std::ios::out);
	if (file.is_open() == true)
	{
		file << texture_classes_.size() << std::endl;
		for (unsigned int i=0; i<data_sample_hierarchy.size(); ++i)
		{
			file << texture_classes_[i] << "\t" << data_sample_hierarchy[i].size() << std::endl;
//			std::cout << texture_classes[i] << ":" << std::endl;
			if (data_sample_hierarchy[i].size() == 0)
				std::cout << "Warning: class " << texture_classes_[i] << " does not contain any objects." << std::endl;
			for (unsigned int j=0; j<data_sample_hierarchy[i].size(); ++j)
			{
				for (unsigned int k=0; k<data_sample_hierarchy[i][j].size(); ++k)
					if (data_sample_hierarchy[i][j][k] == -1)		// invalid entry (caused by files not numbered consecutively)
						data_sample_hierarchy[i][j].erase(data_sample_hierarchy[i][j].begin()+k);

				file << "\t" << data_sample_hierarchy[i][j].size() << std::endl;
//				std::cout << "    O" << j+1 << ":" << std::endl;
				if (data_sample_hierarchy[i][j].size() == 0)
					std::cout << "Warning: class " << texture_classes_[i] << ", object " << j+1 << " does not contain any samples." << std::endl;
				for (unsigned int k=0; k<data_sample_hierarchy[i][j].size(); ++k)
				{
//					std::cout << "          I" << k+1 << ": " << data_sample_hierarchy[i][j][k] << std::endl;
					file << "\t\t" << data_sample_hierarchy[i][j][k] << std::endl;
				}
			}
		}
	}
	else
		std::cout << "Error: could not open file " << filename << "." << std::endl;
	file.close();

	// check data_hierarchy for correct size
	int count = 0;
	for (unsigned int class_index=0; class_index<data_sample_hierarchy.size(); ++class_index)
		for (uint o=0; o<data_sample_hierarchy[class_index].size(); ++o)
			count += data_sample_hierarchy[class_index][o].size();
	assert(count == number_samples);
}

void create_train_data::load_data_hierarchy(std::string filename, DataHierarchyType& data_sample_hierarchy)
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
	else
		std::cout << "Error: could not open file " << filename << "." << std::endl;
	file.close();
}

void create_train_data::load_filenames_gt_attributes(std::string filename, std::map<std::string, std::vector<float> >& filenames_gt_attributes)
{
	const int attribute_number = 17;
	DataHierarchyType data_sample_hierarchy;
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
					std::string image_filename;
					file >> image_filename;
					filenames_gt_attributes[image_filename].resize(attribute_number);
					for (int l=0; l<attribute_number; ++l)
						file >> filenames_gt_attributes[image_filename][l];	// ground truth attribute vector
				}
			}
		}
	}
	else
		std::cout << "Error: could not open file " << filename << "." << std::endl;
	file.close();
}
