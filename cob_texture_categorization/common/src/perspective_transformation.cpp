#include "cob_texture_categorization/perspective_transformation.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/common/pca.h>

#define PI 3.14159265

PerspectiveTransformation::PerspectiveTransformation()
{
}

bool PerspectiveTransformation::normalize_perspective(cv::Mat& image, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, std::vector<float>& plane_coeff, cv::Mat& H_, const double normalized_resolution, const pcl::IndicesPtr indices)
{
	try
	{
		// ========== 1. Determine tangential plane ==========
		// PCA with pcl library
		pcl::PCA<pcl::PointXYZRGB> pca;
		pca.setInputCloud(pointcloud);
		if (indices)
			pca.setIndices(indices);
		Eigen::Matrix3f eigen_vectors_copy = pca.getEigenVectors();

		// fill vectors with pcl pca results
		std::vector<float> eigen3(3);
		for (size_t i=0; i<eigen3.size(); ++i)
			eigen3[i] = (float)eigen_vectors_copy(i,2);
		Eigen::Vector4f meanpoint = pca.getMean();

		// set z direction
		if(eigen3[2]<0)
			for(int i=0;i<3;i++)
				eigen3[i] *= -1;

		// parametric plane equation: ax + by + cz - d = 0
		float a = eigen3[0];
		float b = eigen3[1];
		float c = eigen3[2];
		float d = meanpoint[0]*a + meanpoint[1]*b + meanpoint[2]*c;
		plane_coeff.clear();
		plane_coeff.push_back(a);
		plane_coeff.push_back(b);
		plane_coeff.push_back(c);
		plane_coeff.push_back(d);
		//End PCA

//		// todo: check this part
//		float dist_sum=0;
//		int used_points=0;
////		std::vector<float> var_depth;
//		pcl::PointXYZ point;
////		for(int i=0;i<(*depth).rows;i++)
////		{
////			for(int j=0;j<(*depth).cols;j++)
////			{
//				for(unsigned int i=0;i<(*metricpointcloud).size();i++)
//				{
////					if((*depth).at<float>(i,j)>0)
//					if((*metricpointcloud).points[i].z>0)
//					{
////						var_depth.push_back(depth.at<float>(i,j));
////						point = (*input_cloud2)[i*640+j];
//						point.z = (*metricpointcloud).points[i].z;
//						point.x = (*metricpointcloud).points[i].x;
//						point.y = (*metricpointcloud).points[i].y;
////						std::cout<<point<<" "<<point.z<<" "<<(float)point.z<<std::endl;
//						if(point.z==point.z)
//						{
//							dist_sum = dist_sum + std::abs(((-a*point.x-b*point.y+d)/c)-point.z);
//							used_points++;
//						}
//					}
//				}
////			}
//		if(false == (((dist_sum/used_points)*10)<1))
//		{
////			std::cout<<"No transformation found:"<<std::endl;
//			return false;
//		}

		// ========== 2. compute parameter representation of plane, construct plane coordinate system and compute transformation from camera frame (x,y,z) to plane frame (x,y,z) ==========
		// a) parameter form of plane equation
		// choose two arbitrary points on the plane
		cv::Point3d p1, p2;
		if (a==0. && b==0. && c!=0)
		{
			p1.x = 0;
			p1.y = 0;
			p1.z = -d/c;
			p2.x = 1;
			p2.y = 0;
			p2.z = -d/c;
		}
		else if (a==0. && c==0. && b!=0)
		{
			p1.x = 0;
			p1.y = -d/b;
			p1.z = 0;
			p2.x = 1;
			p2.y = -d/b;
			p2.z = 0;
		}
		else if (b==0. && c==0. && a!=0)
		{
			p1.x = -d/a;
			p1.y = 0;
			p1.z = 0;
			p2.x = -d/a;
			p2.y = 1;
			p2.z = 0;
		}
		else if (a==0. && c!=0)
		{
			p1.x = 0;
			p1.y = 0;
			p1.z = -d/c;
			p2.x = 1;
			p2.y = 0;
			p2.z = -d/c;
		}
		else if (b==0. && c!=0)
		{
			p1.x = 0;
			p1.y = 0;
			p1.z = -d/c;
			p2.x = 0;
			p2.y = 1;
			p2.z = -d/c;
		}
		else if (c==0. && a!=0)
		{
			p1.x = -d/a;
			p1.y = 0;
			p1.z = 0;
			p2.x = -d/a;
			p2.y = 0;
			p2.z = 1;
		}
		else if(c!=0)
		{
			p1.x = 0;
			p1.y = 0;
			p1.z = -d/c;
			p2.x = 1;
			p2.y = 0;
			p2.z = (-d-a)/c;
		}
		else
		{
			std::cout << "Error in plane equation: division by 0." << std::endl;
			return false;
		}
		// compute two normalized directions

		// todo: use first two eigenvectors for good alignment with image borders

		cv::Point3d dirS, dirT, normal(a,b,c);
		double lengthNormal = cv::norm(normal);
		// if (c<0.)
		// lengthNormal *= -1;
		normal.x /= lengthNormal;
		normal.y /= lengthNormal;
		normal.z /= lengthNormal;
		dirS = p2-p1;
		double lengthS = cv::norm(dirS);
		dirS.x /= lengthS;
		dirS.y /= lengthS;
		dirS.z /= lengthS;
		dirT.x = normal.y*dirS.z - normal.z*dirS.y;
		dirT.y = normal.z*dirS.x - normal.x*dirS.z;
		dirT.z = normal.x*dirS.y - normal.y*dirS.x;
		double lengthT = cv::norm(dirT);
		dirT.x /= lengthT;
		dirT.y /= lengthT;
		dirT.z /= lengthT;

		// b) define plane coordinate system
		// plane coordinate frame has origin p1 and x-axis=dirS, y-axis=dirT, z-axis=normal

		// c) compute transformation from camera frame (x,y,z) to plane frame (x,y,z)
		cv::Mat t = (cv::Mat_<double>(3,1) << p1.x, p1.y, p1.z);
		cv::Mat R = (cv::Mat_<double>(3,3) << dirS.x, dirT.x, normal.x, dirS.y, dirT.y, normal.y, dirS.z, dirT.z, normal.z);
		cv::Mat RTt = R.t()*t;

		// ========== 3. select data segment and compute final transformation of camera coordinates to scaled and centered plane coordinates ==========
		// Get matched points to compute transformation matrix
//		const double max_distance_to_camera = 30.0;
		std::vector<cv::Point2f> points_camera, points_plane;
		cv::Point2f min_plane(1e20,1e20), max_plane(-1e20,-1e20);
		for(int v=0; v<image.rows; ++v)
		{
			for(int u=0; u<image.cols; ++u)
			{
				// only use valid points for estimating the transform (black image regions are masked out)
				const cv::Vec3b& color = image.at<cv::Vec3b>(v,u);
				if (color.val[0]==0 && color.val[1]==0 && color.val[2]==0)
					continue;

				// points with nan values cannot be utilized
				const pcl::PointXYZRGB& point = (*pointcloud)[v*image.cols+u];
				if (point.x!=point.x || point.y!=point.y || point.z!=point.z)
					continue;

//				// distance to camera has to be below a maximum distance
//				if (point.x*point.x + point.y*point.y + point.z*point.z > max_distance_to_camera*max_distance_to_camera)
//					continue;

				// determine max and min x and y coordinates of the plane
				cv::Mat point_camera = (cv::Mat_<double>(3,1) << point.x, point.y, point.z);
				cv::Mat point_plane = R.t()*point_camera - RTt;

				points_plane.push_back(cv::Point2f(point_plane.at<double>(0),point_plane.at<double>(1)));	// metric coordinates in tangetial plane's coordinate system (z=0)
				points_camera.push_back(cv::Point2f(u,v));		// pixel coordinates in image

				if (min_plane.x>(float)point_plane.at<double>(0))
				min_plane.x=(float)point_plane.at<double>(0);
				if (max_plane.x<(float)point_plane.at<double>(0))
				max_plane.x=(float)point_plane.at<double>(0);
				if (min_plane.y>(float)point_plane.at<double>(1))
				min_plane.y=(float)point_plane.at<double>(1);
				if (max_plane.y<(float)point_plane.at<double>(1))
				max_plane.y=(float)point_plane.at<double>(1);
			}
		}

		// resize image to fit all of the warped image
		cv::Mat image_original = image.clone();
		image = cv::Mat((max_plane.y-min_plane.y)*normalized_resolution, (max_plane.x-min_plane.x)*normalized_resolution, CV_8UC3);

		// ========== 4. find homography between image plane and plane coordinates ==========
		// a) collect point correspondences
		if (points_camera.size()<100)
			return false;
		std::vector<cv::Point2f> correspondence_points_camera, correspondence_points_plane;
		double step = std::max(1.0, (double)points_camera.size()/100.0);
		cv::Point2f camera_image_plane_offset = cv::Point2f((max_plane.x+min_plane.x)/2.f - (double)image.cols/(2*normalized_resolution), (max_plane.y+min_plane.y)/2.f - (double)image.rows/(2*normalized_resolution));
		for(double i=0;i<points_plane.size();i+=step)
		{
			correspondence_points_camera.push_back(points_camera[(int)i]);
			correspondence_points_plane.push_back(normalized_resolution*(points_plane[(int)i]-camera_image_plane_offset));
		}
		// b) compute homography
		cv::Mat H = findHomography(correspondence_points_camera, correspondence_points_plane);

		// ========== 5. warp perspective ==========
		cv::warpPerspective(image_original, image, H, image.size());
		H_ = H.inv(cv::DECOMP_LU);

//		cv::Mat inverse = H.inv(cv::DECOMP_LU);
//		cv::Mat testt = cv::Mat::zeros(workimage.rows, workimage.cols, CV_8UC3);
//		cv::warpPerspective((*source), testt, inverse, workimage.size());
//		cv::imshow("TEST", *source);
//		cv::imshow("imgtest", testt);
//		cv::imshow("imgtest2", imgbevore);
//		cv::waitKey(10000);
		return true;
	}catch(...)
	{
		std::cout<<"Error in perspective transform."<<std::endl;
		return false;
	}
	return false;

}
