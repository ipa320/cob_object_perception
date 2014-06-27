#include <cob_texture_categorization/texture_categorization.h>



#include "cob_texture_categorization/create_lbp.h"
#include "cob_texture_categorization/splitandmerge.h"
#include "cob_texture_categorization/texture_features.h"
#include "cob_texture_categorization/compute_textures.h"
#include "cob_texture_categorization/depth_image.h"
#include "cob_texture_categorization/segment_trans.h"
#include "cob_texture_categorization/perspective_transformation.h"
#include "cob_texture_categorization/create_train_data.h"
#include "cob_texture_categorization/train_svm.h"
#include "cob_texture_categorization/predict_svm.h"
#include "cob_texture_categorization/color_parameter.h"
#include "cob_texture_categorization/train_ml.h"
#include "cob_texture_categorization/run_meanshift_test.h"
#include "cob_texture_categorization/attribute_learning.h"

#include <iostream>
#include <fstream>
#include <dirent.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

//#include <pcl_ros/point_cloud.h>
//#include <pcl/impl/point_types.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

//#include <pcl_ros/point_cloud.h>






#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;





TextCategorizationNode::TextCategorizationNode(ros::NodeHandle nh) :
node_handle_(nh)
{
//	camera_matrix_received_ = false;



	// subscribers

	it_ = 0;
	sync_input_ = 0;

	it_ = new image_transport::ImageTransport(node_handle_);
	colorimage_sub_.subscribe(*it_, "colorimage_in", 1);
	pointcloud_sub_.subscribe(node_handle_, "pointcloud_in", 1);

//DAS ist ein test

	//segmented_pointcloud_  = nh.subscribe("/surface_classification/segmented_pointcloud", 1, &TextCategorizationNode::segmented_pointcloud_callback, this);


//	sync_input_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >(30);
//	sync_input_->connectInput(colorimage_sub_, pointcloud_sub_);
//	sync_input_->registerCallback(boost::bind(&TextCategorizationNode::inputCallback, this, _1, _2));

	// database tests
	//TextCategorizationNode::inputCallbackNoCam();
	attributeLearningDatabaseTest();

}


void TextCategorizationNode::segmentationCallback(const std_msgs::String::ConstPtr& msg)
{
	std::cout<<"TEST OK!!!!!!!!!!!!!!!!!!"<<std::endl;
}



TextCategorizationNode::~TextCategorizationNode()
{
	if (it_ != 0)
		delete it_;
	if (sync_input_ != 0)
		delete sync_input_;
}

void TextCategorizationNode::init()
{
//		Display normals of transforamtion
//   	coordinatesystem = node_handle_.advertise<visualization_msgs::MarkerArray>("markertest", 1 );
////    cloudpub = node_handle_.advertise<PointCloud> ("points2", 1);
//    pub_cloud = node_handle_.advertise<sensor_msgs::PointCloud2> ("cloud", 1);

}

void TextCategorizationNode::attributeLearningDatabaseTest()
{
	std::string path_database = "/media/SAMSUNG/rmb/datasetTextur/texture_database/";							// path to database
	std::string data_file_name = "/home/rbormann/git/care-o-bot/cob_object_perception/cob_texture_categorization/common/files/farhadi2009/features/ipa_database.txt";		//Pfad zu Speicherort der Featurevektoren

	std::cout << "Loading base features from file ...\n";
	AttributeLearning al;
	cv::Mat feature_matrix, attribute_matrix;
	create_train_data::DataHierarchyType data_hierarchy;
	al.loadTextureDatabaseBaseFeatures(data_file_name, feature_matrix, attribute_matrix, data_hierarchy);
	std::cout << "Loading base features from file finished.\n";

	al.crossValidation(10, feature_matrix, attribute_matrix, data_hierarchy);
}

void TextCategorizationNode::inputCallbackNoCam()
{


	//Computes trainingdata for training of klassification method. uses texture database
	//Saves data in file to hardcoded path

//	std::string path_traindata = "/media/SAMSUNG/rmb/datasetTextur/A_Klassification_Data/train_data/";			//Pfad zu Trainingsdaten
//	std::string path_testdata = "/media/SAMSUNG/rmb/datasetTextur/A_Klassification_Data/test_data/";			//Pfad zu Testdaten
	std::string path_database = "/media/SAMSUNG/rmb/datasetTextur/texture_database/";							// path to database
	std::string path_save_location = "/media/SAMSUNG/rmb/datasetTextur/feature_files/";		//Pfad zu Speicherort der Featurevektoren


//	create_train_data testdata = create_train_data();									// Berechnet den Featurevektor und den einen Labelvektor zum Testen
//	testdata.compute_data(&path_testdata, 2,&path_save_location, 146);
//
//	create_train_data trainingdata = create_train_data();									// Berechnet den Featurevektor und den einen Labelvektor zum Trainieren
//	trainingdata.compute_data(&path_traindata, 1, &path_save_location, 1135);

//	create_train_data database_data = create_train_data();									// computes feature and label matrices of the provided database
//	database_data.compute_data(&path_database, 0, &path_save_location, 1281);

	//Train and predict with NN
	train_ml ml;
	//double gam =0;																		// Trainiert anhand des Trainingsvektors, testet anhand des Testvektors und gibt Ergebnis aus
	//ml.run_ml(gam, &path_save_location);
	cv::Mat feature_matrix, label_matrix;
	create_train_data::DataHierarchyType data_hierarchy;
	ml.load_texture_database_features(path_save_location, feature_matrix, label_matrix, data_hierarchy);
	ml.cross_validation(10, feature_matrix, label_matrix, data_hierarchy);


	//Train and predict with SVM
																							//Einfaches Training und Auswertung mit SVM
//			train_svm traintest = train_svm();
//			traintest.run_training(&path_traindata,&path_testdata, gam, 0, &path_save_location);
//
//			predict_svm prediction = predict_svm();
//			prediction.run_prediction(&path_save_location);




		//	TEST SVM																		//Training und Auswertung der SVM mit unterschiedlichen Gamma werten
//		for(double gam = 0.01; gam<=5; gam=gam+0.2)
//		{
//		//	for(double val = 0.1; val<=10;val=val+0.5)
//		//	{
//			std::string data = "/home/rmb-dh/Test_dataset/training_data.yml";
//			std::string label = "/home/rmb-dh/Test_dataset/train_data_respons.yml";
//			train_svm traintest = train_svm();
//			traintest.run_training(&path_traindata,&path_testdata, gam, 0, &path_save_location);
//
//			predict_svm prediction = predict_svm();
//			prediction.run_prediction(&path_save_location);
//		//	}
//		}



//-----------------------------------Ab hier alter Code -----------------------------------

//	ROS_INFO("Input Callback No Cam");
//	compute_textures test = compute_textures();
//	test.compute_textures_all();
//	test.compute_textures_one();
//	test.compute_test_data();

//	cv::Mat image = cv::imread("/home/rmb-dh/datasetTextur/Texture_database/Wood/Wood_10_01.JPG");
//	cv::Mat image = cv::imread("/home/rmb-dh/datasetTextur/Texture_database/Cracker/Cracker_06_01.JPG");
//	cv::Mat image = cv::imread("/home/rmb-dh/datasetTextur/Texture_database/Styrofoam/Styrofoam_04_01.JPG");
//
//		struct feature_results results;
//	    		struct color_vals color_results;
//	    		color_parameter color = color_parameter();
//	    		color.get_color_parameter(image, &results);
//
//	    		std::cout<<results.colorfulness<<"colorfullness"<<std::endl;
//	    		std::cout<<results.dom_color<<"dom color"<<std::endl;
//	    		std::cout<<results.dom_color2<<"dom color2"<<std::endl;
//
//	    		cv::waitKey(100);











////	TEST K-NEIGHBOR
//	for(double gam = 5; gam<=5; gam=gam+1)
//	{
//	train_kneighbor kn = train_kneighbor();
//	kn.run_training(gam);
//	std::cout<< "Used val:"<<gam<<std::endl;
//}

//	cv::waitKey(10);
}

/// callback for the incoming  data stream
void TextCategorizationNode::inputCallback(const sensor_msgs::Image::ConstPtr& color_image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
{
	ROS_INFO("Input Callback");
	// convert color image to cv::Mat
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;
	convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);










/// convert depth data to cv::Mat
//	cv::Mat depth(480, 640, CV_32F);
//	depth_image dimage = depth_image();
//	dimage.get_depth_image(pointcloud_msg, &depth);
//	cv::imshow("3D",depth);
//	cv::moveWindow("3D", 800,600);



///	Filter to smooth depthimage

	/// Medianfilter
//		dimage.medianfilter(&depth);

	///	Morphological closeing
//		dimage.close_operation(&depth);
//		cv::imshow("smoothed depth", depth);

//		std::vector<float> var_depth_first;
//		for(int i=0;i<depth.rows;i++)
//		{
//			for(int j=0;j<depth.cols;j++)
//			{
//				if(depth.at<float>(i,j)!=0)
//				{
//				var_depth_first.push_back(depth.at<float>(i,j));
//				}
//			}
//		}
//		cv::Scalar means_first, stds_first;
//				cv::meanStdDev(var_depth_first, means_first, stds_first);
//				std::cout<<means_first<<"meansfirst "<<stds_first<<"stdsfirst "<<std::endl;


//	Image segmentation
//		Run meanshift with rgbxy and rgbxyd
//		std::vector < std::vector<cv::Mat> > segmented_regions;
//		run_meanshift_test segmentation_test = run_meanshift_test();
//		segmentation_test.run_test(&color_image, depth, &segmented_regions);
//
//
//		for(int i=0;i<segmented_regions.size();i++)
//		{
//			int j = cv::countNonZero((segmented_regions[i][1]));
//			if(j<600)
//			{
//				segmented_regions.erase(segmented_regions.begin()+i);
//			}
//		}
//		std::cout<<segmented_regions.size()<<"segmented size"<<std::endl;


// 		Imagetransformation
//		cv::imshow("test", color_image);
//		std::vector<float> plane_coeff;

//		for(int i=0;i<segmented_regions.size();i=i+10)
//		{
//			p_transformation transform = p_transformation();
//			transform.run_pca(&color_image, &depth, pointcloud_msg, &marker, &plane_coeff);
//			cv::imshow("segment", segmented_regions[2][0]);
//			transform.run_pca(&segmented_regions[2][0], &depth, pointcloud_msg, &marker);
//		}
//			cv::imshow("transformed region", color_image);
//			cv::imshow("transformed depth", depth);

//			float dist_sum=0;
//			int used_points=0;
//			pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//			pcl::fromROSMsg(*pointcloud_msg, *input_cloud);
////			std::vector<float> var_depth;
//			pcl::PointXYZRGB point;
//			for(int i=0;i<depth.rows;i++)
//			{
//				for(int j=0;j<depth.cols;j++)
//				{
//					if(depth.at<float>(i,j)>0)
//					{
////					var_depth.push_back(depth.at<float>(i,j));
//
//						point = (*input_cloud)[i*640+j];
////						std::cout<<point<<" "<<point.z<<" "<<(float)point.z<<std::endl;
//						if(point.z==point.z)
//						{
//						dist_sum = dist_sum + std::abs(((-plane_coeff[0]*point.x-plane_coeff[1]*point.y+plane_coeff[3])/plane_coeff[2])-point.z);
//						used_points++;
//						}
//					}
//
//				}
//			}
//			std::cout<<point<<"point"<<std::endl;
//			std::cout<<dist_sum<<"absoluter_abstand "<<(dist_sum/used_points)*10<<"normierter abstand "<<used_points<<"points "<<std::endl;
//			cv::Scalar means, stds;
//			cv::meanStdDev(var_depth, means, stds);
//			std::cout<<means<<"means "<<stds<<"stds "<<std::endl;

//		display normals of transformation
//			coordinatesystem.publish(marker);
////		 cloudpub.publish(msg);
//			pub_cloud.publish(pointcloud_msg);







//	cv::Mat test1 = cv::imread("/home/rmb-dh/obst.jpg"); //TEST
//	cv::Mat test2 = cv::imread("/home/rmb-dh/strasse.jpg"); //TEST
//	cv::Mat test3 = cv::imread("/home/rmb-dh/strasse2.jpg"); //TEST
//	cv::Mat test4 = cv::imread("/home/rmb-dh/stadt.jpg"); //TEST
//	cv::Mat test5 = cv::imread("/home/rmb-dh/test4.jpg"); //TEST
//	cv::Mat test6 = cv::imread("/home/rmb-dh/test5.jpg"); //TEST
//	resize(test4, test4, cv::Size(), 0.2, 0.2, cv::INTER_CUBIC);

	// do something useful with code located in common/src
//	cv::Mat gray_image, dx;
//	cv::cvtColor(color_image, gray_image, CV_BGR2GRAY);
//	cv::Sobel(gray_image, dx, -1, 1, 0, 3);
//	double lbp_hist[10];






	//LBP only
//	create_lbp lbp = create_lbp();
//	lbp.create_lbp_class(test4, 1, 8, false, lbp_hist);
//	for(int i=0;i<10;i++)
//	{
//		std::cout << lbp_hist[i]<<"--" << i << "- ";
//	}std::cout <<std::endl;


	//Split and Merge with LBP

//	for(int i=0; i<newimg.rows-2;i++)
//	{for(int j=0;j<newimg.cols;j++)
//	{for(int rgb=0;rgb<3;rgb++)
//	{
//		newimg.at<cv::Vec3b>(i,j)[rgb]=255;
//	}
//	}
//	}
//
	cv::imshow("original", color_image);
	splitandmerge test = splitandmerge();
	cv::Mat pic1 = test.categorize(color_image);
//	cv::Mat pic2 = test.categorize(test2);
//	cv::Mat pic3 = test.categorize(test3);
//	cv::Mat pic4 = test.categorize(test4);
//	cv::Mat pic5 = test.categorize(test5);
//	cv::Mat pic6 = test.categorize(test6);
	cv::imshow("sam1", pic1);
//	cv::moveWindow("sam1", 0,0);
//	cv::imshow("sam2", pic2);
//	cv::moveWindow("sam2", 1380,0);
//	cv::imshow("sam3", pic3);
//	cv::moveWindow("sam3", 740,0);
//	cv::imshow("sam4", pic4);
//	cv::moveWindow("sam4", 0,480);
//	cv::imshow("sam5", pic5);
//	cv::moveWindow("sam5", 1380,480);
//	cv::imshow("sam6", pic6);
//	cv::moveWindow("sam6", 740,480);
//	cv::imshow("sam6o", test6);
//	cv::imshow("orig1", test1);
//	cv::imshow("orig2", test2);
//	cv::imshow("orig3", test3);
//	cv::imshow("orig4", test4);
//	cv::imshow("orig5", test5);


//	cv::Mat picstream = test.categorize(color_image);
//	cv::imshow("image", color_image);
//	cv::imshow("image2", picstream);

//	texture_features edge = texture_features();
//	edge.primitive_size(test1);



//	compute_textures test = compute_textures();
//	test.compute_textures_all();


//	cv::imshow("gray image", gray_image);
//	cv::imshow("dx image", dx);
	cv::waitKey(10);

//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::fromROSMsg(*pointcloud_msg, *cloud);
}


/// Converts a color image message to cv::Mat format.
bool TextCategorizationNode::convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
{
	try
	{
		image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("ObjectCategorization: cv_bridge exception: %s", e.what());
		return false;
	}
	image = image_ptr->image;

	return true;
}

unsigned long TextCategorizationNode::ProjectXYZ(double x, double y, double z, int& u, int& v)
{
	cv::Mat XYZ(4, 1, CV_64FC1);
	cv::Mat UVW(3, 1, CV_64FC1);

	x *= 1000;
	y *= 1000;
	z *= 1000;

	double* d_ptr = XYZ.ptr<double>(0);
	d_ptr[0] = x;
	d_ptr[1] = y;
	d_ptr[2] = z;
	d_ptr[3] = 1.;

	UVW = color_camera_matrix_ * XYZ;

	d_ptr = UVW.ptr<double>(0);
	double du = d_ptr[0];
	double dv = d_ptr[1];
	double dw = d_ptr[2];

	u = cvRound(du/dw);
	v = cvRound(dv/dw);

	return 1;
}

void TextCategorizationNode::calibrationCallback(const sensor_msgs::CameraInfo::ConstPtr& calibration_msg)
{
	if (camera_matrix_received_ == false)
	{
		//	pointcloud_height_ = calibration_msg->height;
		//	pointcloud_width_ = calibration_msg->width;
		cv::Mat temp(3,4,CV_64FC1);
		for (int i=0; i<12; i++)
			temp.at<double>(i/4,i%4) = calibration_msg->P.at(i);
		//		std::cout << "projection_matrix: [";
		//		for (int v=0; v<3; v++)
		//			for (int u=0; u<4; u++)
		//				std::cout << temp.at<double>(v,u) << " ";
		//		std::cout << "]" << std::endl;
		color_camera_matrix_ = temp;
		camera_matrix_received_ = true;
	}
}


int main (int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "texture_categorization");


	// Create a handle for this node, initialize node
	ros::NodeHandle nh;






	// Create and initialize an instance of Object
	TextCategorizationNode texture_categorization(nh);
	texture_categorization.init();




//	ros::Subscriber segmented_pointcloud_;
//	segmented_pointcloud_ = nh.subscribe("/surface_classification/segmented_pointcloud", 1, TextCategorizationNode::segmentationCallback);


	ros::spin();

	return (0);
}
