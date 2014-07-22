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

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

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


// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>



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

//	it_ = new image_transport::ImageTransport(node_handle_);
//	colorimage_sub_.subscribe(*it_, "colorimage_in", 1);
//	pointcloud_sub_.subscribe(node_handle_, "pointcloud_in", 1);

//DAS ist ein test

	segmented_pointcloud_  = nh.subscribe("/surface_classification/segmented_pointcloud", 1, &TextCategorizationNode::segmented_pointcloud_callback, this);


//	sync_input_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >(30);
//	sync_input_->connectInput(colorimage_sub_, pointcloud_sub_);
//	sync_input_->registerCallback(boost::bind(&TextCategorizationNode::inputCallback, this, _1, _2));

	// database tests
//	inputCallbackNoCam();
//	attributeLearningDatabaseTestFarhadi();
//	attributeLearningDatabaseTestHandcrafted();

}
struct segment_position{
	int segment;
	cv::Point2f position;
};
void TextCategorizationNode::segmented_pointcloud_callback(const cob_surface_classification::SegmentedPointCloud2& msg2)
{

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pixel_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr metric_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(msg2.pointcloud, *cloud);
		cv::Mat segment_img;
		cv::Mat depth(480, 640, CV_32F);
		std::vector<float> plane_coeff;
		visualization_msgs::MarkerArray marker;
		std::vector<cv::Mat> segment_vec;
		segment_vec.clear();
		cv::Mat undefined_cluster;				//Contains all small segments
		undefined_cluster = cv::Mat::zeros(480, 640, CV_8UC3);
		cv::Mat seg_whole;						//Image to show all segments of depthsegmentation
		seg_whole = cv::Mat::zeros(480,640,CV_8UC3);
		std::vector<cv::Mat> segment_edges;


		std::vector<segment_position> seg_pos_vec;

		////Get original image
		cv::Mat orig_img;
		orig_img = cv::Mat::zeros(480,640,CV_8UC3);
		for(unsigned int i=0; i<msg2.clusters.size();i++)
		{
			for(unsigned int j=0; j<msg2.clusters[i].array.size();j++)
			{
				int x = (msg2.clusters[i].array[j])%640;
				int y = (int)floor((msg2.clusters[i].array[j])/641);
				orig_img.at<cv::Vec3b>(y,x)[0]=(*cloud).points[msg2.clusters[i].array[j]].b;
				orig_img.at<cv::Vec3b>(y,x)[1]=(*cloud).points[msg2.clusters[i].array[j]].g;
				orig_img.at<cv::Vec3b>(y,x)[2]=(*cloud).points[msg2.clusters[i].array[j]].r;
			}
		}
		cv::Mat orig_img_draw = orig_img.clone();
		int count = 1;
		std::vector<cv::Point2f> schwerepunkt;
		////Transform Segments of Depthsegmentation if possible
		for(unsigned int i=0; i<msg2.clusters.size();i++)
		{
			schwerepunkt.clear();
			if(msg2.clusters[i].array.size()>2000)
			{

				segment_img = cv::Mat::zeros(480,640, CV_8UC3);
				cv::Mat segment_img2 = cv::Mat::zeros(480,640, CV_8UC3);
				cv::Mat binary_img;
				binary_img = cv::Mat::zeros(480,640,CV_32F);
				depth = cv::Mat::zeros(480,640, CV_32F);
				(*pixel_cloud).clear();
				(*metric_cloud).clear();
				for(unsigned int j=0; j<msg2.clusters[i].array.size();j++)
				{

					////Fill pixel cloud
					int x = (msg2.clusters[i].array[j])%640;
					int y = (int)floor((msg2.clusters[i].array[j])/641);

					pcl::PointXYZ point;
					point.z = (*cloud).points[msg2.clusters[i].array[j]].z;
					point.x=x;
					point.y=y;
					pixel_cloud->push_back(point);
					////Fill metric cloud
					point.x = (*cloud).points[msg2.clusters[i].array[j]].x;
					point.y = (*cloud).points[msg2.clusters[i].array[j]].y;
					metric_cloud->push_back(point);
					////Create image of segment
//					segment_img.at<cv::Vec3b>(y,x)[0]=(*cloud).points[msg2.clusters[i].array[j]].b;
//					segment_img.at<cv::Vec3b>(y,x)[1]=(*cloud).points[msg2.clusters[i].array[j]].g;
//					segment_img.at<cv::Vec3b>(y,x)[2]=(*cloud).points[msg2.clusters[i].array[j]].r;
					////Create depthimage of segment
					depth.at<float>(y,x)=(*cloud).points[msg2.clusters[i].array[j]].z;
					binary_img.at<float>(y,x)=255;

				}

				////dilate binary image for reducing of artefacts
				int dilation_size = 1;
				int dilation_type = cv::MORPH_RECT;//MORPH_RECT MORPH_CROSS  MORPH_ELLIPSE;
				cv::Mat element = cv::getStructuringElement( dilation_type, cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
						                                       cv::Point( dilation_size, dilation_size ) );
				cv::dilate( binary_img, binary_img, element );

				////Create new image out of dilated binary image
				for(int iy=0;iy<480;iy++)
				{
					for(int ix=0;ix<640;ix++)
					{
						if(binary_img.at<int>(iy,ix)>0)
						{
							segment_img.at<cv::Vec3b>(iy,ix)[0]=orig_img_draw.at<cv::Vec3b>(iy,ix)[0];
							segment_img.at<cv::Vec3b>(iy,ix)[1]=orig_img_draw.at<cv::Vec3b>(iy,ix)[1];
							segment_img.at<cv::Vec3b>(iy,ix)[2]=orig_img_draw.at<cv::Vec3b>(iy,ix)[2];
							schwerepunkt.push_back(cv::Point(ix,iy));
						}
					}
				}
				cv::Mat edges = cv::Mat::zeros(480,640, CV_8UC1);
				////Create data for visualistation
				binary_img.convertTo(binary_img,CV_8U,255.0/(255));
				int nonZero = countNonZero(binary_img);
				cv::Canny(binary_img, edges , 254, 255, 3);
				std::vector<std::vector<cv::Point> > contours;
				std::vector<cv::Vec4i> hierarchy;
				cv::findContours(edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
				cv::Mat drawing = cv::Mat::zeros( edges.size(), CV_8UC3 );
				segment_edges.push_back(edges);
				std::vector<cv::Point> used_contours;
				for(unsigned int cont1=0;cont1<contours.size();cont1++)
				{
					for(unsigned int cont2=0; cont2<contours[cont1].size();cont2++)
					{
						if((int)contours[cont1].size()>40)
						{
							used_contours.push_back(contours[cont1][cont2]);
						}
					}
				}

				bool usefull_3D_data = true;

					///  Get the mass centers:
					cv::Point2f mc;
					/// Get min rect around contours
					cv::RotatedRect rec;
//					imwrite( "/home/rmb-dh/Pictures/minArea1.jpg", segment_img );
					rec =  minAreaRect(schwerepunkt);

					mc = rec.center;
					cv::Point2f rect_points[4]; rec.points( rect_points );
					double size_of_rec = rec.size.width * rec.size.height;
					double fuellgrad = (nonZero)/size_of_rec;
//					for( int j = 0; j < 4; j++ )
//									cv::line( segment_img, rect_points[j], rect_points[(j+1)%4], cvScalar(255,0,0), 1, 8 );
				if(fuellgrad<0.3)
				{
					usefull_3D_data =false;


				}else{
					////Drawn Lines in orig image and rect in contours image
					for(unsigned int ci = 0; ci< contours.size(); ci++ )
					{
						if((int)contours[ci].size()>30)
						{
							  cv::Scalar color = cv::Scalar(30,144,255);
							  cv::drawContours( orig_img, contours, ci, color, 1, 8, hierarchy, 0, cv::Point() );
						 }
					}
//					for( int j = 0; j < 4; j++ )
//					cv::line( orig_img, rect_points[j], rect_points[(j+1)%4], cvScalar(255,0,0), 1, 8 );

					if(fuellgrad<=0.3)
					{
						  struct segment_position pos;
						  pos.position = mc;
						  pos.segment = count;
						  seg_pos_vec.push_back(pos);
						  count++;
					}else
					{
						int index_bigcont=0;
						for(unsigned int bigcont=1;bigcont<contours.size();bigcont++)
						{
							if(contours[index_bigcont].size()<contours[bigcont].size())
							{
								index_bigcont=bigcont;
							}
						}
//						int pos_bigcont = floor(contours[index_bigcont].size()/2);
						struct segment_position pos;
						pos.position = mc;//contours[index_bigcont][pos_bigcont];
						pos.segment = count;
						seg_pos_vec.push_back(pos);
						count++;
					 }
				}



				if(usefull_3D_data)
				{
				////Compute Transformation and save transformed image in segment_vec
				p_transformation transform_segment = p_transformation();
				transform_segment.run_pca(&segment_img, &depth, pixel_cloud, metric_cloud, &marker, &plane_coeff);

				cv::Mat newimg = segment_img.clone();
				segment_vec.push_back(newimg);


				}else{
					for(unsigned int j=0; j<msg2.clusters[i].array.size();j++)
						{
								int x = (msg2.clusters[i].array[j])%640;
								int y = (int)floor((msg2.clusters[i].array[j])/641);
								undefined_cluster.at<cv::Vec3b>(y,x)[0]=(*cloud).points[msg2.clusters[i].array[j]].b;
								undefined_cluster.at<cv::Vec3b>(y,x)[1]=(*cloud).points[msg2.clusters[i].array[j]].g;
								undefined_cluster.at<cv::Vec3b>(y,x)[2]=(*cloud).points[msg2.clusters[i].array[j]].r;
						}
				}

			}else{
				for(unsigned int m=0; m<msg2.clusters[i].array.size();m++)
				{
					int x = (msg2.clusters[i].array[m])%640;
					int y = (int)floor((msg2.clusters[i].array[m])/641);
					undefined_cluster.at<cv::Vec3b>(y,x)[0]=(*cloud).points[msg2.clusters[i].array[m]].b;
					undefined_cluster.at<cv::Vec3b>(y,x)[1]=(*cloud).points[msg2.clusters[i].array[m]].g;
					undefined_cluster.at<cv::Vec3b>(y,x)[2]=(*cloud).points[msg2.clusters[i].array[m]].r;
				}
			}
		}
		//Add unsegmented areas as one image to vector of segmented images
		cv::Mat work_segment;
		cvtColor( undefined_cluster, work_segment, CV_BGR2GRAY );
		int nonZeros = cv::countNonZero(work_segment);
		if((nonZeros)/(640*480)>0.2)
		{

			segment_vec.push_back(undefined_cluster);
			struct segment_position pos;
			pos.position = cv::Point2f(10,10);
			pos.segment = count;
			seg_pos_vec.push_back(pos);
		}




		////Reduce segment on necessary area
		if(segment_vec.size()>1){
		for(unsigned int i=0;i<segment_vec.size();i++)
		{
			std::cout<<segment_vec[i].size()<<std::endl;
//			cv::imshow("segments", segment_vec[i]);
//			cv::waitKey(10000);

			std::vector <cv::Point> seg_points;
			cv::Mat work_segment = segment_vec[i];
			for(int pi=0; pi<480;pi++)
			{
				for(int pj=0; pj<640;pj++)
				{
					if(work_segment.at<cv::Vec3b>(pi,pj)[0]!=0 || work_segment.at<cv::Vec3b>(pi,pj)[1]!=0 || work_segment.at<cv::Vec3b>(pi,pj)[2]!=0)
					{
						seg_points.push_back(cv::Point(pj,pi));
					}
				}
			}
//			imwrite( "/home/rmb-dh/Pictures/minArea2.jpg", work_segment );
			cv::RotatedRect rec;
			if(seg_points.size()>3)
			{
				rec =  minAreaRect(seg_points);
			}
			cv::Mat M, rotated, new_segment;
			// get angle and size from the bounding box
			float angle = rec.angle;
			cv::Size rect_size = rec.size;
			if (rec.angle < -45.)
			{
				angle += 90.0;
				swap(rect_size.width, rect_size.height);
			}
			bool use_segment = true;
			if(segment_vec[i].cols>100 && segment_vec[i].rows>100)
			{
				M = cv::getRotationMatrix2D(rec.center, angle, 1.0);
				cv::warpAffine(work_segment, rotated, M, work_segment.size(), cv::INTER_CUBIC);
;
				if(rotated.empty())
				{
				cv::Mat test(480,640,CV_32F);
				cvtColor(rotated, test, CV_BGR2GRAY);
//				imwrite( "/home/rmb-dh/Pictures/reduced.jpg", rotated );
				cv::getRectSubPix(rotated, rect_size, rec.center, new_segment);
				}else{
					use_segment=false;
				}

			}
			if(use_segment)
				segment_vec[i] = new_segment;
		}
		}

//		for(unsigned int i=0;i<segment_vec.size();i++)
//		{
//			cv::imshow("image", segment_vec[i]);
//			cv::waitKey(100000);
//		}




		////Segment with split and merge
		std::vector<cv::Mat> swap_vec;
		splitandmerge seg_step_two = splitandmerge();
//		for(unsigned int i=0; i< segment_vec.size(); i++)
//		{

//		for(double ik=0;ik<1;ik= ik +0.1)
//		{
//		seg_step_two.categorize(orig_img, &swap_vec, i);

			swap_vec.clear();
//			if(segment_vec[i].rows >50 && segment_vec[i].cols>50)
//			{

//				seg_step_two.categorize(segment_vec[i], &swap_vec, 0);

				seg_step_two.categorize(orig_img_draw, &swap_vec,0);
//			}
//			std::cout<<i<<" mergevalue"<<std::endl;

			for(unsigned int j=0;j<swap_vec.size();j++)
			{

				cv::imshow("splitres",swap_vec[j]);
				cv::imshow("org", orig_img);
				cv::waitKey(100000);


			}
			swap_vec.clear();
//		}

//		}


		////Compute Features of Segments
		std::vector<struct feature_results> segment_features;
		struct feature_results results;
		cv::Mat img_seg;

		for(unsigned int i=0;i<segment_vec.size();i++)
		{
//			imwrite( "/home/rmb-dh/Pictures/features.jpg", segment_vec[i] );
			img_seg = segment_vec[i];
			color_parameter color = color_parameter();
			color.get_color_parameter(img_seg, &results);
			texture_features edge = texture_features();
			cv::Mat dummy(480,640,CV_32F);
			edge.primitive_size(&img_seg, &results, &dummy);
			segment_features.push_back(results);
		}

		////Create matrix for classifikation
		cv::Mat feature_mat;
		feature_mat = cv::Mat::zeros(segment_features.size(), 16, CV_32FC1);
		for(unsigned int sample_index=0;sample_index<segment_features.size();sample_index++)
		{
			results = segment_features[sample_index];
			feature_mat.at<float>(sample_index, 0) = results.colorfulness; // 3: colorfulness
			feature_mat.at<float>(sample_index, 1) = results.dom_color; // 4: dominant color
			feature_mat.at<float>(sample_index, 2) = results.dom_color2; // 5: dominant color2
			feature_mat.at<float>(sample_index, 3) = results.v_mean; //6: v_mean
			feature_mat.at<float>(sample_index, 4) = results.v_std; // 7: v_std
			feature_mat.at<float>(sample_index, 5) = results.s_mean; // 8: s_mean
			feature_mat.at<float>(sample_index, 6) = results.s_std; // 9: s_std
			feature_mat.at<float>(sample_index, 7) = results.avg_size; // 10: average primitive size
			feature_mat.at<float>(sample_index, 8) = results.prim_num; // 11: number of primitives
			feature_mat.at<float>(sample_index, 9) = results.prim_strength; // 12: strength of primitives
			feature_mat.at<float>(sample_index, 10) = results.prim_regularity; // 13: regularity of primitives
			feature_mat.at<float>(sample_index, 11) = results.contrast; // 14: contrast:
			feature_mat.at<float>(sample_index, 12) = results.line_likeness; // 15: line-likeness
			//	Nicht implementiert	    	feature_mat.at<float>(count,13) = results.roughness; // 16: 3D roughness
			feature_mat.at<float>(sample_index, 13) = results.direct_reg; // 17: directionality/regularity
			feature_mat.at<float>(sample_index, 14) = results.lined; // 18: lined
			feature_mat.at<float>(sample_index, 15) = results.checked; // 19: checked
		}

//		Run classifikation with SVM
		cv::Mat prediction_results;
	 	CvSVM SVM;
	    SVM.load("/home/rmb-dh/datasetTextur/yamlfiles/svm.yml", "svm");
	    SVM.predict(feature_mat,prediction_results);


	    ////Write Segment type
	    std::vector<std::string> classes;
	    create_train_data get_classes = create_train_data();
	    classes = get_classes.get_texture_classes();
	    std::string s;
	    for(int i=0;i<prediction_results.rows;i++)
	    {
	    	s.clear();
	    	s = classes[prediction_results.at<float>(i,0)];
	    	putText(orig_img, s, seg_pos_vec[i].position,
	    						 			cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,205), 1, CV_AA);
	    }

//		cv::imshow("orig", orig_img);
//		cv::waitKey(10);

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

void TextCategorizationNode::attributeLearningDatabaseTestFarhadi()
{
	// === using the farhadi attributes that are learned from base features
	std::string path_database = "/media/SAMSUNG/rmb/datasetTextur/texture_database/";							// path to database
	std::string data_file_name = "/home/rbormann/git/care-o-bot/cob_object_perception/cob_texture_categorization/common/files/farhadi2009/features/ipa_texture_database/ipa_database_2fb.txt";		//Pfad zu Speicherort der Featurevektoren
	std::string data_path = "/home/rbormann/git/care-o-bot/cob_object_perception/cob_texture_categorization/common/files/farhadi2009/features/ipa_texture_database/";

	// attribute learning
	std::cout << "Loading base features, attributes and class hierarchy from file ...\n";
	AttributeLearning al;
	cv::Mat base_feature_matrix, ground_truth_attribute_matrix, class_label_matrix;
	create_train_data::DataHierarchyType data_hierarchy;
	al.loadTextureDatabaseBaseFeatures(data_file_name, 9688, 17, base_feature_matrix, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy);
	std::cout << "Loading base features, attributes and class hierarchy from file finished.\n";

	int folds = 20;
	std::vector< std::vector<int> > preselected_train_indices;
	std::vector<cv::Mat> attribute_matrix_test_data, class_label_matrix_test_data;
	al.crossValidation(folds, base_feature_matrix, ground_truth_attribute_matrix, data_hierarchy, 0, true, class_label_matrix, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data);
	al.saveAttributeCrossValidationData(data_path, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data);
	//al.loadAttributeCrossValidationData(data_path, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data);

	// final classification: NN learned with labeled attribute data from the training set and tested with the predicted attributes
//	//std::cout << "Loading labeled attribute features from file ...\n";
//	//al.loadTextureDatabaseLabeledAttributeFeatures(data_file_name, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy);
//	//std::cout << "Loading labeled attribute features from file finished.\n";

	train_ml ml;
	//ml.cross_validation(folds, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy);		// use this version if training and test data shall be drawn from the same data matrix
	ml.cross_validation(folds, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data);	// use this if test data is stored in a different matrix than training data, e.g. because training data comes from the labeled attributes and test data is computed attributes
}

void TextCategorizationNode::attributeLearningDatabaseTestHandcrafted()
{
	// === using the hand crafted attributes
//	std::string path_database = "/media/SAMSUNG/rmb/datasetTextur/texture_database/";			// path to database
	std::string path_database = "/home/rmb-dh/datasetTextur/test_data/";			// path to database
//	std::string path_save_location = "/media/SAMSUNG/rmb/datasetTextur/feature_files/";		// path to save data
//	std::string data_file_name = "/home/rbormann/git/care-o-bot/cob_object_perception/cob_texture_categorization/common/files/feature_files/ipa_database_handcrafted_2fb.txt";		//Pfad zu Speicherort der Featurevektoren
//	std::string feature_files_path = "/home/rbormann/git/care-o-bot/cob_object_perception/cob_texture_categorization/common/files/feature_files/"; // path to save data
	std::string data_file_name = "/home/rmb-dh/git/care-o-bot/cob_object_perception/cob_texture_categorization/common/files/feature_files/ipa_database_handcrafted_2fb.txt";		//Pfad zu Speicherort der Featurevektoren
	std::string feature_files_path = "/home/rbormann/git/care-o-bot/cob_object_perception/cob_texture_categorization/common/files/feature_files/"; // path to save data

//	// compute 16 texture attributes on the ipa texture database
	create_train_data database_data;									// computes feature and label matrices of the provided database
	//database_data.compute_data(path_database, feature_files_path, 1281);
	//return;

	// attribute cross-validation
	cv::Mat base_feature_matrix, ground_truth_attribute_matrix, computed_attribute_matrix, class_label_matrix;
	create_train_data::DataHierarchyType data_hierarchy;
	train_ml ml;
	AttributeLearning al;
	std::cout << "Loading base features, attributes and class hierarchy from file ...\n";
	// option 1: pre-computed in MATLAB:
//	al.loadTextureDatabaseBaseFeatures(data_file_name, 16, 17, computed_attribute_matrix, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy);
//	cv::Mat temp = ground_truth_attribute_matrix.clone();
//	ground_truth_attribute_matrix.create(temp.rows, temp.cols-1, temp.type());
//	for (int r=0; r<temp.rows; ++r)
//		for (int c=0; c<16; ++c)
//			ground_truth_attribute_matrix.at<float>(r,c) = temp.at<float>(r,c+(c<13 ? 0 : 1));
	// option 2: computed with this program
	database_data.load_texture_database_features(feature_files_path, base_feature_matrix, ground_truth_attribute_matrix, computed_attribute_matrix, class_label_matrix, data_hierarchy);
	std::cout << "Loading base features, attributes and class hierarchy from file finished.\n";

	int folds = 20;
	std::vector< std::vector<int> > preselected_train_indices;
	std::vector<cv::Mat> attribute_matrix_test_data, class_label_matrix_test_data;
	al.crossValidation(folds, computed_attribute_matrix, ground_truth_attribute_matrix, data_hierarchy, 0, true, class_label_matrix, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data);
	al.saveAttributeCrossValidationData(feature_files_path, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data);
	//al.loadAttributeCrossValidationData(data_path, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data);

	// final classification: NN learned with labeled attribute data from the training set and tested with the predicted attributes
	//ml.cross_validation(folds, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy);		// use this version if training and test data shall be drawn from the same data matrix
	ml.cross_validation(folds, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data);	// use this if test data is stored in a different matrix than training data, e.g. because training data comes from the labeled attributes and test data is computed attributes
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
//	testdata.compute_data(path_testdata, path_save_location, 146, 2);
//
//	create_train_data trainingdata = create_train_data();									// Berechnet den Featurevektor und den einen Labelvektor zum Trainieren
//	trainingdata.compute_data(path_traindata, path_save_location, 1135, 1);

//	create_train_data database_data = create_train_data();									// computes feature and label matrices of the provided database
//	database_data.compute_data(path_database, path_save_location, 1281);

	//Train and predict with NN
	train_ml ml;
	//double gam =0;																		// Trainiert anhand des Trainingsvektors, testet anhand des Testvektors und gibt Ergebnis aus
	//ml.run_ml(gam, &path_save_location);
	cv::Mat base_feature_matrix, ground_truth_attribute_matrix, computed_attribute_matrix, class_label_matrix;
	create_train_data::DataHierarchyType data_hierarchy;
	create_train_data database_data;
	database_data.load_texture_database_features(path_save_location, base_feature_matrix, ground_truth_attribute_matrix, computed_attribute_matrix, class_label_matrix, data_hierarchy);
	ml.cross_validation(10, computed_attribute_matrix, class_label_matrix, data_hierarchy);


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
