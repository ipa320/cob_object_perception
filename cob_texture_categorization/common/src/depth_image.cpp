#include "cob_texture_categorization/depth_image.h"
#include <cob_texture_categorization/texture_categorization.h>

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

depth_image::depth_image()
{
}
// Medianfiltering of depthimage.
void depth_image::medianfilter(cv::Mat *depth_image)
{
		cv::Mat dst(480, 640, CV_8U);
		for(int i=0;i<640;i++)
		{
			for(int j = 0; j<480;j++)
			{

				dst.at<uchar>(j,i)=round((*depth_image).at<float>(j,i)*42.5);
			}
		}

		int MAX_KERNEL_LENGTH = 31;
//		for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
//		    { cv::medianBlur ( dst, dst, i ); }
//		    cv::medianBlur ( dst, dst, 3 );
//		    cv::medianBlur ( dst, dst, 7 );
		    cv::medianBlur ( dst, dst, 21 );



		for(int i=0;i<640;i++)
		{
			for(int j = 0; j<480;j++)
			{

				(*depth_image).at<float>(j,i)=dst.at<uchar>(j,i)/42.5;
			}
		}
}

void depth_image::close_operation(cv::Mat *depth_image)
{
		cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT, cv::Size(5, 5) , cv::Point(-1,-1));
		cv::morphologyEx(*depth_image, *depth_image, cv::MORPH_CLOSE, element );
		element = cv::getStructuringElement( cv::MORPH_RECT, cv::Size(10, 10) , cv::Point(-1,-1));
		cv::morphologyEx(*depth_image, *depth_image, cv::MORPH_CLOSE, element );
		//	element = cv::getStructuringElement( cv::MORPH_RECT, cv::Size(15, 15) , cv::Point(-1,-1));
		//	cv::morphologyEx(dst, dst, cv::MORPH_CLOSE, element );
		cv::imshow("median",*depth_image);
}

// Creates depthimage
void depth_image::get_depth_image(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg, cv::Mat *depth)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*pointcloud_msg, *cloud);

cv::Mat image_new(480, 640, CV_8UC3);
	for(int i=0;i<640;i++)
	{
		for(int j = 0; j<480;j++)
		{
			image_new.at<cv::Vec3b>(j,i)[0]=0;
			image_new.at<cv::Vec3b>(j,i)[1]=0;
			image_new.at<cv::Vec3b>(j,i)[2]=0;
			(*depth).at<float>(j,i)=0;
		}
	}


	for (int i=0; i< 307200; i++){

	if((*cloud).points[i].x!=(*cloud).points[i].x&& (*cloud).points[i].y!=(*cloud).points[i].y&&(*cloud).points[i].z!=(*cloud).points[i].z)
	{
		int x = i%640;
		int y = (int)floor(i/641);
		int z = floor((*cloud).points[i].z);//*51);

		if(z>255)z=255;
		image_new.at<cv::Vec3b>(y,x)[0]=0;
		image_new.at<cv::Vec3b>(y,x)[1]=0;
		image_new.at<cv::Vec3b>(y,x)[2]=255;
//		image_new_gray.at<cv::Vec3b>(y,x)=0;
		(*depth).at<float>(y,x)=0;

	}else{
		int x = i%640;
		int y = (int)floor(i/641);
		int z = floor((*cloud).points[i].z);//*85);

		if(z>255)z=255;

		image_new.at<cv::Vec3b>(y,x)[0]=z;
		image_new.at<cv::Vec3b>(y,x)[1]=z;
		image_new.at<cv::Vec3b>(y,x)[2]=z;
//		image_new_gray.at<cv::Vec3b>(y,x)=z;
		(*depth).at<float>(y,x)=(*cloud).points[i].z;

	}
	}

}
