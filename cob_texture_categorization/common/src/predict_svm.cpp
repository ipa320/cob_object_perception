#include "cob_texture_categorization/predict_svm.h"


#include "cob_texture_categorization/create_train_data.h"
#include <cob_texture_categorization/texture_categorization.h>

#include "cob_texture_categorization/compute_textures.h"
#include "cob_texture_categorization/create_lbp.h"
#include "cob_texture_categorization/splitandmerge.h"
#include "cob_texture_categorization/texture_features.h"
#include "cob_texture_categorization/write_xml.h"
#include "cob_texture_categorization/color_parameter.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>


#include <iostream>
#include <fstream>
#include <dirent.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


predict_svm::predict_svm()
{
}

void predict_svm::run_prediction(std::string *path_)
{
//	std::vector<cv::Mat> image;
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Alufoil_06_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Asphalt_06_03.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Bookshelf_06_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Bread_06_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Brick_02_03.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Broccoli_06_03.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Carpet_04_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Cauliflower_02_03.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/CD_05_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Chocolate_07_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Clock_03_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Coconut_05_03.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Coffee_01_08.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Concrete_04_03.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Corduroy_03_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Cork_04_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Cotton_06_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Cracker_05_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Cup_04_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Flakes_04_03.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Flour_04_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Foam_03_03.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Football_01_03.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Fork_04_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Fur_05_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Granite_06_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Grapes_02_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Ingrain_05_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Jalousie_03_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Kiwi_07_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Knife_04_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Leather_04_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Lemon_02_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Lime_06_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Linen_04_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Marble_06_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Mouse_03_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Orange_06_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Parsley_07_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Pasta_03_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Pavingstone_03_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/PCKeyboard_03_03.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Pineapple_05_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Plate_05_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Rice_05_02.jpg"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Sand_02_04.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Smarties_05_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Sponge_05_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Spoon_04_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Styrofoam_04_03.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Telephone_06_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Texwallpaper_07_03.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Tiles_06_03.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Tomato_12_01.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Varnished_05_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Washingmachine_05_02.JPG"));
//	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/Wood_04_03.JPG"));
////	image.push_back(cv::imread("/home/rmb-dh/Tomate.jpg"));
//
//	cv::Mat train_data(image.size(),16, CV_32FC1);
//
//	for(int count=0;count<image.size();count++)
//	{
//
//		struct feature_results results;
//  		color_parameter color = color_parameter();
//		try{
//  		color.get_color_parameter(image[count], &results);
//		}catch(...){
//			std::cout<<"color"<<count<<std::endl;
//		}
//		texture_features edge = texture_features();
//    	try{
//		edge.primitive_size(image[count], &results);
//    	}catch(...)
//    	{std::cout<<"text"<<count<<std::endl;
//
//    	}
//		std::cout<<count<<"count"<<std::endl;
//
//    	train_data.at<float>(count,0) = results.colorfulness; // 3: colorfullness
//    	train_data.at<float>(count,1) = results.dom_color; // 4: dominant color
//    	train_data.at<float>(count,2) = results.dom_color2; // 5: dominant color2
//    	train_data.at<float>(count,3) = results.v_mean; //6: v_mean
//    	train_data.at<float>(count,4) = results.v_std; // 7: v_std
//    	train_data.at<float>(count,5) = results.s_mean; // 8: s_mean
//    	train_data.at<float>(count,6) = results.s_std; // 9: s_std
//    	train_data.at<float>(count,7) = results.avg_size; // 10: average primitive size
//    	train_data.at<float>(count,8) = results.prim_num; // 11: number of primitives
//    	train_data.at<float>(count,9) = results.prim_strength; // 12: strength of primitives
//    	train_data.at<float>(count,10) = results.prim_regularity; // 13: regularity of primitives
//    	train_data.at<float>(count,11) = results.contrast; // 14: contrast:
//    	train_data.at<float>(count,12) = results.line_likeness; // 15: line-likeness
////	Nicht implementiert	    	train_data.at<float>(count,13) = results.roughness; // 16: 3D roughness
//    	train_data.at<float>(count,13) = results.direct_reg; // 17: directionality/regularity
//    	train_data.at<float>(count,14) = results.lined; // 18: lined
//    	train_data.at<float>(count,15) = results.checked; // 19: checked
//
//
//
//	}

//	cv::FileStorage fs("/home/rmb-dh/Test_dataset/predict_test_data.yml", cv::FileStorage::WRITE);
//	fs << "test_data" << train_data;

	std::string train_path = *path_ + "test_data.yml";
	cv::Mat train_data;
//	cv::FileStorage fs("/home/rmb-dh/Test_dataset/predict_test_data.yml", cv::FileStorage::READ);
	cv::FileStorage fs(train_path, cv::FileStorage::READ);
	fs["test_data"] >> train_data;

	cv::Mat test_data_label;
	std::string test_label = *path_ + "test_data_label.yml";
//	cv::FileStorage fstl("/home/rmb-dh/Test_dataset/test_data_label.yml", cv::FileStorage::READ);
	cv::FileStorage fstl(test_label, cv::FileStorage::READ);
	fstl["test_label"] >> test_data_label;



//	std::cout<<train_data<<std::cout;

	cv::Mat prediction_results;

#if CV_MAJOR_VERSION == 2
 	CvSVM SVM;
    SVM.load("datasetTextur/yamlfiles/svm.yml", "svm");
    SVM.predict(train_data,prediction_results);
#else
    cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
    svm->load("datasetTextur/yamlfiles/svm.yml");
    svm->predict(train_data,prediction_results);
#endif
//    std::cout<<prediction_results.size();
//    std::cout<<" Results Predictionnum "<<":  "<<prediction_results<<"Results prediction"<<std::endl;
//    std::cout<<prediction_results.type()<<"type";
    int right=0;
    		int wrong=0;
    for(int i =0;i<prediction_results.rows;i++)
    {
//    std::cout<<" Results Predictionnum "<<i<<":  "<<prediction_results.at<float>(i)<<"Results prediction"<<std::endl;
    	if(test_data_label.at<float>(i,0)==prediction_results.at<float>(i))
    	{
    		right++;
    	}else{
    		wrong++;
    	}
    }
    std::cout<<"True:"<<right<<"  "<<"False:" << wrong<<std::endl;
    double sum = right+wrong;
    double percentage =  (100/sum) ;
    std::cout<<"Correct classified: "<<percentage*right<<"%"<<std::endl;


}
