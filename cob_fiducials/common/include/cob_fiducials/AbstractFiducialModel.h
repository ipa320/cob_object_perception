#ifndef __IPA_ABSTRACT_FIDUCIAL_MODEL_H__
#define __IPA_ABSTRACT_FIDUCIAL_MODEL_H__

//#include "../../../../../cob_object_perception_intern/windows/src/PreCompiledHeaders/StdAfx.h"
#ifdef __LINUX__
	#include "cob_vision_utils/VisionUtils.h"
	#include "cob_fiducials/FiducialDefines.h"
	#include "cob_fiducials/AbstractFiducialParameters.h"

	#include "tinyxml.h"
#else
	#include "cob_perception_common/cob_vision_utils/common/include/cob_vision_utils/VisionUtils.h"
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/FiducialDefines.h"
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/AbstractFiducialParameters.h"

	#include "cob_object_perception_intern/windows/src/extern/TinyXml/tinyxml.h"	
#endif

#include <boost/shared_ptr.hpp>
#include <opencv/cv.h>
// todo: remove after debugging
#include <opencv/highgui.h>

#include <map>

namespace ipa_Fiducials
{

/// Define smart pointer type for object model
class AbstractFiducialModel;
typedef boost::shared_ptr<AbstractFiducialModel> AbstractFiducialModelPtr;
/// @class FiducialModelPi
///
/// A concrete class to represent a fiducial
class __DLL_LIBFIDUCIALS__ AbstractFiducialModel
{
public:

	//*******************************************************************************
	// AbstractFiducial interface implementation
	//*******************************************************************************
	virtual ~AbstractFiducialModel(){};

	unsigned long Init(cv::Mat& camera_matrix, std::string directory_and_filename, cv::Mat extrinsic_matrix = cv::Mat())
	{
		if (camera_matrix.empty())
		{
			std::cerr << "ERROR - AbstractFiducialModel::Init" << std::endl;
			std::cerr << "\t [FAILED] Camera matrix not initialized" << std::endl;
			return ipa_Utils::RET_FAILED;
		}
		m_camera_matrix = camera_matrix.clone();

		m_extrinsic_XYfromC = cv::Mat::zeros(4, 4, CV_64FC1);
		if (extrinsic_matrix.empty())
		{
			// Unit matrix
			for (int i=0; i<3; i++)
				m_extrinsic_XYfromC.at<double>(i,i) = 1.0;
		}
		else
		{
			for (int i=0; i<3; i++)
				for (int j=0; j<4; j++)
				{
					m_extrinsic_XYfromC.at<double>(i,j) = extrinsic_matrix.at<double>(i,j);
				}
			m_extrinsic_XYfromC.at<double>(3,3) = 1.0;
		}
		
		return LoadParameters(directory_and_filename);
	};

	unsigned long ApplyExtrinsics(cv::Mat& rot_CfromO, cv::Mat& trans_CfromO)
	{
		cv::Mat frame_CfromO = cv::Mat::zeros(4, 4, CV_64FC1);

		// Copy ORIGINAL rotation and translation to frame
		for (int i=0; i<3; i++)
		{
			frame_CfromO.at<double>(i, 3) = trans_CfromO.at<double>(i,0);
			for (int j=0; j<3; j++)
			{
				frame_CfromO.at<double>(i,j) = rot_CfromO.at<double>(i,j);
			}
		}
		frame_CfromO.at<double>(3,3) = 1.0;

		cv::Mat frame_XYfromO = m_extrinsic_XYfromC * frame_CfromO;

		// Copy MODIFIED rotation and translation to frame
		for (int i=0; i<3; i++)
		{
			trans_CfromO.at<double>(i, 0) = frame_XYfromO.at<double>(i,3);
			for (int j=0; j<3; j++)
			{
				rot_CfromO.at<double>(i,j) = frame_XYfromO.at<double>(i,j);
			}
		}

		return ipa_Utils::RET_OK;
	}

	/// Locates the fiducial within the image and inferes the camera pose from it
	/// @param image scene image
	/// @param vec_pose Vector of poses from all detected tags relative to the camera
	/// @return <code>RET_FAILED</code> if no tag could be detected
	/// <code>RET_OK</code> on success
	virtual unsigned long GetPose(cv::Mat& image, std::vector<t_pose>& vec_pose_CfromO) = 0;

	unsigned long GetSharpnessMeasure(const cv::Mat& image, t_pose pose_CfromO, const AbstractFiducialParameters& fiducial_parameters, double& sharpness_measure)
	{
		if (fiducial_parameters.m_id == -1)
		{
			std::cerr << "ERROR - AbstractFiducialModel::GetSharpnessMeasure" << std::endl;
			std::cerr << "\t [FAILED] Could not find general fiducial parameters to the provided id." << std::endl;
			return ipa_Utils::RET_FAILED;
		}

		// select image region for sharpness analysis
		cv::Mat point3d_marker = cv::Mat::zeros(4,3,CV_64FC1);
		point3d_marker.at<double>(0,0) = fiducial_parameters.m_sharpness_pattern_area_rect3d.x + fiducial_parameters.m_offset.x;	// upper left
		point3d_marker.at<double>(0,1) = -fiducial_parameters.m_sharpness_pattern_area_rect3d.y + fiducial_parameters.m_offset.y;
		point3d_marker.at<double>(1,0) = fiducial_parameters.m_sharpness_pattern_area_rect3d.x + fiducial_parameters.m_sharpness_pattern_area_rect3d.width + fiducial_parameters.m_offset.x;	// upper right
		point3d_marker.at<double>(1,1) = -fiducial_parameters.m_sharpness_pattern_area_rect3d.y + fiducial_parameters.m_offset.y;
		point3d_marker.at<double>(2,0) = fiducial_parameters.m_sharpness_pattern_area_rect3d.x + fiducial_parameters.m_sharpness_pattern_area_rect3d.width + fiducial_parameters.m_offset.x;	// lower right
		point3d_marker.at<double>(2,1) = -fiducial_parameters.m_sharpness_pattern_area_rect3d.y - fiducial_parameters.m_sharpness_pattern_area_rect3d.height + fiducial_parameters.m_offset.y;
		point3d_marker.at<double>(3,0) = fiducial_parameters.m_sharpness_pattern_area_rect3d.x + fiducial_parameters.m_offset.x;	// lower left
		point3d_marker.at<double>(3,1) = -fiducial_parameters.m_sharpness_pattern_area_rect3d.y - fiducial_parameters.m_sharpness_pattern_area_rect3d.height + fiducial_parameters.m_offset.y;
		std::vector<cv::Point> sharpness_area(4);
		cv::Point min_point(image.cols-1, image.rows-1), max_point(0,0);
		for (int i=0; i<4; ++i)
		{
			cv::Mat point3d_camera = pose_CfromO.rot * point3d_marker.row(i).t() + pose_CfromO.trans;
			cv::Mat point2d_camera = m_camera_matrix * point3d_camera;
			sharpness_area[i].x = std::max(0, std::min(image.cols-1, cvRound(point2d_camera.at<double>(0)/point2d_camera.at<double>(2))));
			sharpness_area[i].y = std::max(0, std::min(image.rows-1, cvRound(point2d_camera.at<double>(1)/point2d_camera.at<double>(2))));
			if (min_point.x > sharpness_area[i].x)
				min_point.x = sharpness_area[i].x;
			if (min_point.y > sharpness_area[i].y)
				min_point.y = sharpness_area[i].y;
			if (max_point.x < sharpness_area[i].x)
				max_point.x = sharpness_area[i].x;
			if (max_point.y < sharpness_area[i].y)
				max_point.y = sharpness_area[i].y;
		}
		cv::Mat roi = image.rowRange(min_point.y, max_point.y);
		roi = roi.colRange(min_point.x, max_point.x);

		// compute sharpness measure
		cv::Mat temp, gray_image;
		cv::cvtColor(roi, temp, CV_BGR2GRAY);
		cv::normalize(temp, gray_image, 0, 255, cv::NORM_MINMAX);
		cv::imshow("gray_image", gray_image);

		cv::Mat image_copy = image.clone();
		for (int i=0; i<4; ++i)
			cv::line(image_copy, sharpness_area[i], sharpness_area[(i+1)%4], CV_RGB(0,255,0), 2);
		for (unsigned int i=0; i<sharpness_area.size(); ++i)
			sharpness_area[i] -= min_point;			// map sharpness_area into the roi
		cv::Mat dx, dy;
		cv::Sobel(gray_image, dx, CV_32FC1, 1, 0, 3);
		cv::Sobel(gray_image, dy, CV_32FC1, 0, 1, 3);
		dx = cv::abs(dx);
		dy = cv::abs(dy);
		double score = 0.;
		int pixel_count = 0;
		for (int v=0; v<roi.rows; ++v)
		{
			for (int u=0; u<roi.cols; ++u)
			{
				if (cv::pointPolygonTest(sharpness_area, cv::Point2f(u,v), false) > 0)
				{
					score += dx.at<float>(v,u) + dy.at<float>(v,u);
					++pixel_count;
//					cv::circle(image_copy, cv::Point(u+min_point.x, v+min_point.y), 1, CV_RGB(255,0,0),1);
				}
			}
		}
//		if (pixel_count > 0)
//			score /= sqrt((double)pixel_count);
		double camera_dist = cv::norm(pose_CfromO.trans, cv::NORM_L2);
		score *= camera_dist;

//		double score = (cv::sum(cv::abs(dx)) + cv::sum(cv::abs(dy))).val[0] / ((double)image.cols*image.rows);
		std::cout << "pixel_count=" << pixel_count << " \t sharpness score=" << score << std::endl;

		cv::imshow("sharpness area", image_copy);
		cv::waitKey(10);

		return ipa_Utils::RET_OK;
	}

	// Gets the camera matrix
	// @return 3x3 camera matrix (fx 0 cx, 0 fy cy, 0 0 1) 
	cv::Mat GetCameraMatrix()
	{
		return m_camera_matrix;
	};

	// Set the camera matrix
	// @param camera_matrix 3x3 camera matrix (fx 0 cx, 0 fy cy, 0 0 1) 
	unsigned long SetCameraMatrix(cv::Mat camera_matrix)
	{
		m_camera_matrix = camera_matrix.clone();
		return ipa_Utils::RET_OK;
	}

	// Gets the distortion coeffs
	// @return 1x4 distortion coeffs matrix (k1,k2,p1,p2)
	cv::Mat GetDistortionCoeffs()
	{
		if (m_dist_coeffs.empty())
			return cv::Mat::zeros(1, 4, CV_64FC1);
		return m_dist_coeffs;
	};

	// Sets the distortion coeffs
	// @param dist_coeffs 1x4 distortion coeffs matrix (k1,k2,p1,p2)
	unsigned long SetDistortionCoeffs(cv::Mat dist_coeffs)
	{
		m_dist_coeffs = dist_coeffs.clone();
		return ipa_Utils::RET_OK;
	}

	// Gets the general fiducial parameters for a certain marker
	// @return general fiducial parameters
	AbstractFiducialParameters GetGeneralFiducialParameters(int marker_id)
	{
		std::map<int, AbstractFiducialParameters>::iterator it = m_general_fiducial_parameters.find(marker_id);
		if (it==m_general_fiducial_parameters.end())
		{
			AbstractFiducialParameters empty;
			empty.m_id = -1;
			return empty;
		}
		return it->second;
	};

	/// Load fiducial-centric coordinates of markers from file
	/// @param directory_and_filename Path to XML filename, where the parameters of all fiducials are stores.
	/// When using ROS, this function is replaced by parsing a launch file
	virtual unsigned long LoadParameters(std::string directory_and_filename) = 0;

	/// Return the name (fiducial type) of the detector
	/// @return fiducial type
	virtual std::string GetType() = 0;
private:

	cv::Mat m_camera_matrix; ///< Intrinsics of camera for PnP estimation
	cv::Mat m_dist_coeffs; ///< Intrinsics of camera for PnP estimation
	cv::Mat m_extrinsic_XYfromC; ///< Extrinsics 4x4 of camera to rotate and translate determined transformation before returning it

protected:
	std::map<int, AbstractFiducialParameters> m_general_fiducial_parameters;	///< map of marker id to some general parameters like offsets
};

} // end namespace ipa_Fiducials

#endif // __IPA_ABSTRACT_FIDUCIAL_MODEL_H__
