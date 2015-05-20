#include "cob_texture_categorization/create_train_data.h"

#include "cob_texture_categorization/compute_textures.h"
#include "cob_texture_categorization/create_lbp.h"
#include "cob_texture_categorization/splitandmerge.h"
#include "cob_texture_categorization/texture_features.h"
#include "cob_texture_categorization/write_xml.h"
#include "cob_texture_categorization/color_parameter.h"

#include <highgui.h>


#include <iostream>
#include <fstream>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/time.h>

#include <fstream>


create_train_data::create_train_data(int database)
{
	if (database==0)		// IPA database
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
	else if (database==1)
	{
		texture_classes_.push_back("generated");
	}
	else if (database==2)		// DTD database
	{
		texture_classes_.push_back("banded");		// 0
		texture_classes_.push_back("blotchy");
		texture_classes_.push_back("braided");
		texture_classes_.push_back("bubbly");
		texture_classes_.push_back("bumpy");
		texture_classes_.push_back("chequered");
		texture_classes_.push_back("cobwebbed");
		texture_classes_.push_back("cracked");
		texture_classes_.push_back("crosshatched");
		texture_classes_.push_back("crystalline");
		texture_classes_.push_back("dotted");		// 10
		texture_classes_.push_back("fibrous");
		texture_classes_.push_back("flecked");
		texture_classes_.push_back("freckled");
		texture_classes_.push_back("frilly");
		texture_classes_.push_back("gauzy");
		texture_classes_.push_back("grid");
		texture_classes_.push_back("grooved");
		texture_classes_.push_back("honeycombed");
		texture_classes_.push_back("interlaced");
		texture_classes_.push_back("knitted");		// 20
		texture_classes_.push_back("lacelike");
		texture_classes_.push_back("lined");
		texture_classes_.push_back("marbled");
		texture_classes_.push_back("matted");
		texture_classes_.push_back("meshed");
		texture_classes_.push_back("paisley");
		texture_classes_.push_back("perforated");
		texture_classes_.push_back("pitted");
		texture_classes_.push_back("pleated");
		texture_classes_.push_back("polka-dotted");		// 30
		texture_classes_.push_back("porous");
		texture_classes_.push_back("potholed");
		texture_classes_.push_back("scaly");
		texture_classes_.push_back("smeared");
		texture_classes_.push_back("spiralled");
		texture_classes_.push_back("sprinkled");
		texture_classes_.push_back("stained");
		texture_classes_.push_back("stratified");
		texture_classes_.push_back("striped");
		texture_classes_.push_back("studded");		// 40
		texture_classes_.push_back("swirly");
		texture_classes_.push_back("veined");
		texture_classes_.push_back("waffled");
		texture_classes_.push_back("woven");
		texture_classes_.push_back("wrinkled");
		texture_classes_.push_back("zigzagged");
	}
}

std::vector<std::string> create_train_data::get_texture_classes()
{
	return texture_classes_;
}

void create_train_data::compute_data_handcrafted(std::string path_database_images, std::string path_save, const std::string& database_identifier)
{
	std::string label_file;
	if (database_identifier.compare("ipa") == 0)
		label_file = "ipa_database.txt";
	else if (database_identifier.compare("dtd") == 0)
		label_file = "dtd_database.txt";

	// load labeled ground truth attributes with relation to each image file
	std::vector<std::string> image_filenames;		// a list of all available database image filenames
	std::map<std::string, std::vector<float> > filenames_gt_attributes;		// corresponding ground truth attributes to each file
	create_train_data::DataHierarchyType data_sample_hierarchy;			// data_sample_hierarchy[class_index][object_index][sample_index] = entry_index in feature data matrix
	std::string path_filenames_gt_attributes = path_save + label_file;
	load_filenames_gt_attributes(path_filenames_gt_attributes, texture_classes_, image_filenames, filenames_gt_attributes, data_sample_hierarchy, database_identifier);
	const int number_pictures = image_filenames.size();

	cv::Mat ground_truth_attribute_matrix = cv::Mat::zeros(number_pictures, filenames_gt_attributes.begin()->second.size(), CV_32FC1);	// matrix of labeled ground truth attributes
	cv::Mat computed_attribute_matrix = cv::Mat::zeros(number_pictures, 17, CV_32FC1);			// matrix of computed attributes
	cv::Mat class_label_matrix = cv::Mat::zeros(number_pictures, 1, CV_32FC1);			// matrix of correct classes
	cv::Mat base_feature_matrix = cv::Mat::zeros(number_pictures, 23, CV_32FC1); // matrix of computed base features

	std::cout << "Computing image features ... " << std::endl;
	std::stringstream accumulated_error_string;
	std::vector<int> errors;
	for(int class_index=0;class_index<(int)texture_classes_.size(); ++class_index)
	{
		for (int object_index=0; object_index<data_sample_hierarchy[class_index].size(); ++object_index)
		{
			for (int image_index=0; image_index<data_sample_hierarchy[class_index][object_index].size(); ++image_index)
			{
				const int sample_index = data_sample_hierarchy[class_index][object_index][image_index];
				std::string name = image_filenames[sample_index];
				std::string image_filename = path_database_images + name;

				// ground truth attributes
				if (filenames_gt_attributes.find(name) != filenames_gt_attributes.end())
				{
					std::cout << name << "\ngt:\t";
					for (unsigned int i=0, j=0; i<filenames_gt_attributes[name].size(); ++i)
						//if (i!=13)	// one attribute (3d roughness) is currently not implemented here, so just leave it out from gt
					{
							ground_truth_attribute_matrix.at<float>(sample_index, j) = filenames_gt_attributes[name][i];
							std::cout << filenames_gt_attributes[name][i] << "\t";
							++j;
					}
					std::cout << std::endl;
				}
				else
				{
					std::cout << "Error: no entry for file '" << name << "' in filenames_gt_attributes." << std::endl;
					accumulated_error_string << "Error: no entry for file '" << name << "' in filenames_gt_attributes." << std::endl;
				}

				// compute features

//				 definition of raw features:
//
//				 1. colorfulness: (raw[0])
//				 raw: 0,1,2 --> 0,1,2
//				 raw: [2,5] --> min(a*raw+b, 5)
//
//				 2. dominant color: (raw[1])
//				 raw: [0,10] --> [0,10] as is
//
//				 3. secondary dominant color: (raw[2])
//				 raw: [0,10] --> [0,10] as is
//
//				 4. value mean: (raw[3])
//				 raw: --> max(1, min(5, a*raw+b))
//
//				 5. value stddev: (raw[4])
//				 raw: --> max(1, min(5, a*raw+b))
//
//				 6. saturation mean: (raw[5])
//				 raw: --> max(1, min(5, a*raw+b))
//
//				 7. saturation stddev: (raw[6])
//				 raw: --> max(1, min(5, a*raw+b))
//
//				 8. average primitive size: (raw[7], raw[8])
//				 raw[8]: 1 --> 1
//				 raw[7],raw[8]: --> max(1, min(5, (a1*raw[7]+b1 + a2*raw[8]+b2)/2) )
//
//				 9. number of primitives: (raw[9], raw[10])
//				 raw[9],raw[10]: --> max(1, min(5, max(a1*raw[9]+b1, a2*raw[10]+b2) ) )
//
//				 10. primitive strength: (raw[11])
//				 raw: --> max(1, min(5, a*raw*raw + b*raw + c ) )
//
//				 11. primitive regularity: (raw[12], raw[13], raw[14])
//				 raw: --> max(1, min(5, a*raw[12] + b*raw[13] + c*raw[14] + d ) )
//
//				 12. contrast: (raw[15])
//				 raw: --> max(1, min(5, a*raw[15]^3 + b*raw[15]^2 + c*raw[15] + d ) )
//
//				 13. line likeness: (raw[16])
//				 raw: [1,4,5] --> [1,4,5] as is
//				 raw: --> max(1, min(5, a*raw[16] + b ) )
//
//				 14. 3d roughness
//				 not implemented for 2d images
//
//				 15. directionality: (raw[17], raw[18], raw[19])
//				 raw[17]: [1] --> [1]
//				 raw[17]: --> max(1, min(5, a*raw[17]+b))
//				 raw[18]: --> max(1, min(5, a*raw[18]+b))
//				 raw[19]: --> max(1, min(5, a*raw[19]+b))
//				 directionality = max(mapped(raw[17]), mapped(raw[18]), mapped(raw[19]), lined, checked)
//
//				 16. lined: (raw[20])
//				 raw[20]: --> max(1, min(5, a*raw[20]+b))
//
//				 17. checked: (raw[21])
//				 raw[21]: --> max(1, min(5, a*raw[21]+b))

				cv::Mat image = cv::imread(image_filename);
//				cv::Mat temp;		// hack: downsizing image
//				cv::resize(image, temp, cv::Size(), 0.25, 0.25, cv::INTER_AREA);
//				image = temp;
				feature_results results;
				cv::Mat raw_features = base_feature_matrix.row(sample_index); // todo: check width
				color_parameter color; // color features
				color.get_color_parameter_new(image, &results, &raw_features);
				texture_features edge; // texture features
				edge.compute_texture_features(image, results, &raw_features);

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
				//	not well implemented (quite random choice)
				computed_attribute_matrix.at<float>(sample_index, 13) = results.roughness; // 16: 3D roughness
				computed_attribute_matrix.at<float>(sample_index, 14) = results.direct_reg; // 17: directionality/regularity
				computed_attribute_matrix.at<float>(sample_index, 15) = results.lined; // 18: lined
				computed_attribute_matrix.at<float>(sample_index, 16) = results.checked; // 19: checked
				std::cout << "comp:\t";
				for (int i = 0; i < computed_attribute_matrix.cols; i++)
				{
					if (computed_attribute_matrix.at<float>(sample_index, i) != computed_attribute_matrix.at<float>(sample_index, i))
					{
						errors.push_back(i);
						computed_attribute_matrix.at<float>(sample_index, i) = 0;
					}
					std::cout << computed_attribute_matrix.at<float>(sample_index, i) << "\t";
				}

				// label
				class_label_matrix.at<float>(sample_index, 0) = class_index;

				std::cout << "\n\n\t\tFeature computation completed: " << (sample_index / (double)number_pictures) * 100 << "%   Picnum " << sample_index << std::endl;
			}
		}
	}

	for (int i = 0; i < (int)errors.size(); i++)
	{
		std::cout << "Error in feature " << errors[i] << "for some sample." << std::endl;
	}

	std::cout << "Errors:\n" << accumulated_error_string.str() << std::endl;

	std::cout << "Finished reading " << image_filenames.size() << " data samples." << std::endl;

	//	Save computed attributes, class labels and ground truth attributes
	save_texture_database_features(path_save, base_feature_matrix, ground_truth_attribute_matrix, computed_attribute_matrix, class_label_matrix, data_sample_hierarchy, image_filenames, database_identifier);

	std::cout << "Feature computation on database completed." << std::endl;
}


void create_train_data::compute_data_cimpoi(std::string path_database_images, std::string path_save, const std::string& database_identifier, bool generateGMM, IfvFeatures::FeatureType feature_type)
{
	// parameters
	const int number_gaussian_centers = 256;
	const int feature_samples_per_image = 500;	//1000	//200
	std::string label_file;
	double image_resize_factor = 0.;
	if (database_identifier.compare("ipa") == 0)
	{
		label_file = "ipa_database.txt";
		image_resize_factor = 0.25;
	}
	else if (database_identifier.compare("dtd") == 0)
	{
		label_file = "dtd_database.txt";
		image_resize_factor = 1.0;
	}

	// read out database files, their hierarchy, and load labeled ground truth attributes to each image file
	std::vector<std::string> image_filenames;		// a list of all available database image filenames
	std::map<std::string, std::vector<float> > filenames_gt_attributes;		// corresponding ground truth attributes to each file
	create_train_data::DataHierarchyType data_sample_hierarchy;		// data_sample_hierarchy[class_index][object_index][sample_index] = entry_index in feature data matrix
	std::string path_filenames_gt_attributes = path_save + label_file;
	load_filenames_gt_attributes(path_filenames_gt_attributes, texture_classes_, image_filenames, filenames_gt_attributes, data_sample_hierarchy, database_identifier);
	const int number_pictures = image_filenames.size();

	// compute or load GMM
	IfvFeatures ifv;
	std::string gmm_filename = path_save + database_identifier + "_gmm_model.yml";
	srand(0);		// keep random numbers reproducible
	if (generateGMM == true)
	{
		// extend filenames by path to database
		std::vector<std::string> image_filenames_full_path(image_filenames.size());
		for(size_t i=0; i<image_filenames.size(); ++i)
			image_filenames_full_path[i] = path_database_images + image_filenames[i];

		// compute and store GMM
		ifv.constructGenerativeModel(image_filenames_full_path, image_resize_factor, feature_samples_per_image, number_gaussian_centers, feature_type);
		ifv.saveGenerativeModel(gmm_filename);
	}
	else
		ifv.loadGenerativeModel(gmm_filename);

	// compute features for each image file
	cv::Mat ground_truth_attribute_matrix = cv::Mat::zeros(number_pictures, filenames_gt_attributes.begin()->second.size(), CV_32FC1);	// matrix of labeled ground truth attributes
	cv::Mat class_label_matrix = cv::Mat::zeros(number_pictures, 1, CV_32FC1);			// matrix of correct classes
	cv::Mat base_feature_matrix = cv::Mat::zeros(number_pictures, 2*ifv.getFeatureDimension(feature_type)*number_gaussian_centers, CV_32FC1); // matrix of computed base features

	srand(0);		// keep random numbers reproducible
	std::cout << "Computing image features ... " << std::endl;
	std::stringstream accumulated_error_string;
	for(int class_index=0;class_index<(int)texture_classes_.size(); ++class_index)
	{
		for (int object_index=0; object_index<data_sample_hierarchy[class_index].size(); ++object_index)
		{
			for (int image_index=0; image_index<data_sample_hierarchy[class_index][object_index].size(); ++image_index)
			{
				const int sample_index = data_sample_hierarchy[class_index][object_index][image_index];
				std::string name = image_filenames[sample_index];
				std::string image_filename = path_database_images + name;

				// ground truth attributes
				if (filenames_gt_attributes.find(name) != filenames_gt_attributes.end())
				{
					std::cout << image_filename << "\ngt:\t";
					for (unsigned int i=0; i<filenames_gt_attributes[name].size(); ++i)
					{
							ground_truth_attribute_matrix.at<float>(sample_index, i) = filenames_gt_attributes[name][i];
							std::cout << filenames_gt_attributes[name][i] << "\t";
					}
					std::cout << std::endl;
				}
				else
				{
					std::cout << "Error: no entry for file '" << name << "' in filenames_gt_attributes." << std::endl;
					accumulated_error_string << "Error: no entry for file '" << name << "' in filenames_gt_attributes." << std::endl;
				}

				// compute IFV base features
				cv::Mat base_features = base_feature_matrix.row(sample_index);
				ifv.computeImprovedFisherVector(image_filename, image_resize_factor, number_gaussian_centers, base_features, feature_type);

				// label
				class_label_matrix.at<float>(sample_index, 0) = class_index;

				std::cout << "\n\t\tFeature computation completed: " << (sample_index / (double)number_pictures) * 100 << "%   Picnum " << sample_index << std::endl;
			}
		}
	}

	std::cout << "Errors:\n" << accumulated_error_string.str() << std::endl;

	std::cout << "Finished reading " << image_filenames.size() << " data samples." << std::endl;

	//	Save computed attributes, class labels and ground truth attributes
	save_texture_database_features(path_save, base_feature_matrix, ground_truth_attribute_matrix, cv::Mat(), class_label_matrix, data_sample_hierarchy, image_filenames, database_identifier);

	std::cout << "Feature computation on database completed." << std::endl;
}


void create_train_data::save_texture_database_features(const std::string path, const cv::Mat& base_feature_matrix, const cv::Mat& ground_truth_attribute_matrix, const cv::Mat& computed_attribute_matrix, const cv::Mat& class_label_matrix, DataHierarchyType& data_sample_hierarchy, const std::vector<std::string>& image_filenames, const std::string& database_identifier)
{
	//	Save computed attributes, class labels and ground truth attributes
	const std::string database_file = path + database_identifier + "_database.yml";
	cv::FileStorage fs(database_file, cv::FileStorage::WRITE);
	fs << "base_feature_matrix" << base_feature_matrix;
	fs << "ground_truth_attribute_matrix" << ground_truth_attribute_matrix;
	fs << "computed_attribute_matrix" << computed_attribute_matrix;
	fs << "class_label_matrix" << class_label_matrix;
	fs.release();

	// store hierarchy and check validity of hierarchical data structure
	const std::string database_hierarchy_file = path + database_identifier + "_database_hierarchy.txt";
	save_data_hierarchy(database_hierarchy_file, data_sample_hierarchy, image_filenames, class_label_matrix.rows);
}

void create_train_data::load_texture_database_features(std::string path, cv::Mat& base_feature_matrix, cv::Mat& ground_truth_attribute_matrix, cv::Mat& computed_attribute_matrix, cv::Mat& class_label_matrix, DataHierarchyType& data_sample_hierarchy, std::vector<std::string>& image_filenames, const std::string& database_identifier)
{
	// load computed attributes, class labels and ground truth attributes
	const std::string database_file = path + database_identifier + "_database.yml";
	std::cout << "Reading file " << database_file << std::endl;
	cv::FileStorage fs(database_file, cv::FileStorage::READ);
	fs["base_feature_matrix"] >> base_feature_matrix;
	fs["ground_truth_attribute_matrix"] >> ground_truth_attribute_matrix;
	fs["computed_attribute_matrix"] >> computed_attribute_matrix;
	fs["class_label_matrix"] >> class_label_matrix;
	fs.release();

	// load class-object-sample hierarchy
	std::string database_hierarchy_file = path + database_identifier + "_database_hierarchy.txt";
	load_data_hierarchy(database_hierarchy_file, data_sample_hierarchy, image_filenames);

	std::cout << "Texture database features loaded." << std::endl;
}

void create_train_data::save_data_hierarchy(const std::string filename, DataHierarchyType& data_sample_hierarchy, const std::vector<std::string>& image_filenames, const int number_samples)
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
					file << "\t\t" << data_sample_hierarchy[i][j][k] << "\t" << image_filenames[data_sample_hierarchy[i][j][k]] << std::endl;
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

void create_train_data::load_data_hierarchy(std::string filename, DataHierarchyType& data_sample_hierarchy, std::vector<std::string>& image_filenames)
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
					std::string filename;
					file >> filename;
					image_filenames.push_back(filename);
				}
			}
		}
	}
	else
		std::cout << "Error: could not open file " << filename << "." << std::endl;
	file.close();
}

void create_train_data::load_filenames_gt_attributes(std::string filename, std::vector<std::string>& class_names, std::vector<std::string>& image_filenames, std::map<std::string, std::vector<float> >& filenames_gt_attributes, DataHierarchyType& data_sample_hierarchy, const std::string& database_identifier)
{
	class_names.clear();
	image_filenames.clear();
	filenames_gt_attributes.clear();
	data_sample_hierarchy.clear();

	int attribute_number = 0;
	if (database_identifier.compare("ipa") == 0)
		attribute_number = 17;
	else if (database_identifier.compare("dtd") == 0)
		attribute_number = 47;
	else
		std::cout << "Error: create_train_data::load_filenames_gt_attributes: unsupported mode selected.";
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
			class_names.push_back(class_name);
			unsigned int object_number=0;
			file >> object_number;
			data_sample_hierarchy[i].resize(object_number);
			for (unsigned int j=0; j<object_number; ++j)
			{
				unsigned int sample_number=0;
				file >> sample_number;
				data_sample_hierarchy[i][j].resize(sample_number);
				for (unsigned int k=0; k<sample_number; ++k)
				{
					// filename
					std::string image_filename;
					file >> image_filename;
					image_filename = class_name + "/" + image_filename;
					image_filenames.push_back(image_filename);
					data_sample_hierarchy[i][j][k] = image_filenames.size()-1;
					// ground truth labels
					filenames_gt_attributes[image_filename].resize(attribute_number);
					while (true)
					{
						std::string tag;
						file >> tag;
						if (database_identifier.compare("ipa") == 0)
						{
							if (tag.compare("labels_ipa17:") == 0)
							{
								for (int l=0; l<attribute_number; ++l)
									file >> filenames_gt_attributes[image_filename][l];	// ground truth attribute vector
							}
							else if (tag.compare("base_farhadi9688:") == 0)
							{
								std::string buffer;
								std::getline(file, buffer);
								break;
							}
							else
							{
								std::cout << "Error: create_train_data::load_filenames_gt_attributes: Unknown tag found." << std::endl;
								getchar();
								break;
							}
						}
						else if (database_identifier.compare("dtd") == 0)
						{
							if (tag.compare("labels_cimpoi47:") == 0)
							{
								for (int l=0; l<attribute_number; ++l)
									file >> filenames_gt_attributes[image_filename][l];	// ground truth attribute vector
								break;
							}
							else
							{
								std::cout << "Error: create_train_data::load_filenames_gt_attributes: Unknown tag found." << std::endl;
								getchar();
								break;
							}
						}
						else
							std::cout << "Error: create_train_data::load_filenames_gt_attributes: unsupported mode selected.";
					}
				}
			}
		}
	}
	else
		std::cout << "Error: could not open file " << filename << "." << std::endl;
	file.close();

	std::cout << "Found " << image_filenames.size() << " image files in database distributed into " << class_names.size() << " classes." << std::endl;
}


void create_train_data::create_dtd_database_file(std::string path_database_images, std::string path_save, std::string label_file)
{
	// generate a list of all available database image filenames
	std::map<std::string, std::vector<std::string> > classwise_image_filenames;
	for(int class_index=0;class_index<(int)texture_classes_.size(); ++class_index)
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
					classwise_image_filenames[texture_classes_[class_index]].push_back(name);
				}
			}
		}
	}
	std::string filename = path_save + label_file;
	std::ofstream file(filename.c_str(), std::ios::out);
	if (file.is_open()==true)
	{
		file << classwise_image_filenames.size() << std::endl;	// class_number
		size_t class_index = 0;
		for (std::map<std::string, std::vector<std::string> >::iterator class_it = classwise_image_filenames.begin(); class_it!=classwise_image_filenames.end(); ++class_it, ++class_index)
		{
			file << class_it->first << "\t" << class_it->second.size() << std::endl;	// class_name and object_number
			std::sort(class_it->second.begin(), class_it->second.end());
			for (size_t i=0; i<class_it->second.size(); ++i)
			{
				file << "\t1\n";	// only one sample per object
				file << "\t\t" << class_it->second[i] << std::endl;
				file << "\t\t\tlabels_cimpoi47:\t";
				for (size_t j=0; j<class_index; ++j)
					file << "0\t";
				file << "1\t";
				for (size_t j=class_index+1; j<47; ++j)
					file << "0\t";
				file << std::endl;
			}
		}
	}
	else
		std::cout << "Error: create_train_data::load_dtd_filenames_gt_attributes: could not open file " << filename << "." << std::endl;
	file.close();
}
