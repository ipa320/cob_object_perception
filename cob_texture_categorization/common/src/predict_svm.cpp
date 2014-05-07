#include "predict_svm.h"


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

void predict_svm::run_prediction()
{
	std::vector<cv::Mat> image;
	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/test/Alufoil_06_01.JPG"));
	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/test/Asphalt_06_03.JPG"));
	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/test/Bookshelf_06_02.JPG"));
	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/test/Bread_06_01.JPG"));
	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/test/Brick_02_03.JPG"));
	image.push_back(cv::imread("/home/rmb-dh/datasetTextur/Test_data/test/Broccoli_06_03.JPG"));

	cv::Mat train_data(6,16, CV_32FC1);

	for(int count=0;count<6;count++)
	{
		struct feature_results results;
  		color_parameter color = color_parameter();
		color.get_color_parameter(image[count], &results);
		texture_features edge = texture_features();
    	edge.primitive_size(image[count], &results);


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



	}





	std::cout<<train_data<<std::cout;

	cv::Mat prediction_results;

 	CvSVM SVM;
    SVM.load("/home/rmb-dh/Test_dataset/svm.yml", "svm");
    SVM.predict(train_data,prediction_results);

    std::cout<<" Results Prediction "<<prediction_results<<"Results prediction"<<std::endl;


}
