#include <cob_vision_utils/StdAfx.h>

#ifdef __LINUX__
	#include "cob_fiducials/AbstractFiducialModel.h"
	#include <opencv2/imgproc/imgproc.hpp>
	#include <opencv2/highgui/highgui.hpp>
	#include <iomanip>
#else
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/AbstractFiducialModel.h"
#endif

using namespace ipa_Fiducials;


unsigned long AbstractFiducialModel::Init(cv::Mat& camera_matrix, std::string directory_and_filename, bool log_or_calibrate_sharpness_measurements, cv::Mat extrinsic_matrix)
{
	if(SetExtrinsics(camera_matrix, extrinsic_matrix) & ipa_Utils::RET_FAILED)
		return ipa_Utils::RET_FAILED;

	m_log_or_calibrate_sharpness_measurements = log_or_calibrate_sharpness_measurements;
		
	return LoadParameters(directory_and_filename);
}

unsigned long AbstractFiducialModel::SetExtrinsics(cv::Mat& camera_matrix, cv::Mat extrinsic_matrix) 
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
		
	return ipa_Utils::RET_OK;
}

unsigned long AbstractFiducialModel::ApplyExtrinsics(cv::Mat& rot_CfromO, cv::Mat& trans_CfromO)
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

	
unsigned long AbstractFiducialModel::GetSharpnessMeasure(const cv::Mat& image, t_pose pose_CfromO, 
	const AbstractFiducialParameters& fiducial_parameters, double& sharpness_measure, 
	double sharpness_calibration_parameter_m, double sharpness_calibration_parameter_n)
{
	if (fiducial_parameters.m_id == -1)
	{
		std::cerr << "ERROR - AbstractFiducialModel::GetSharpnessMeasure" << std::endl;
		std::cerr << "\t [FAILED] Could not find general fiducial parameters to the provided id." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	// 1. select image region for sharpness analysis
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

	// 2. compute sharpness measure
	cv::Mat temp, gray_image;
	cv::cvtColor(roi, temp, CV_BGR2GRAY);
	cv::normalize(temp, gray_image, 0, 255, cv::NORM_MINMAX);

//		cv::imshow("gray_image", gray_image);
//		cv::Mat image_copy = image.clone();
//		for (int i=0; i<4; ++i)
//			cv::line(image_copy, sharpness_area[i], sharpness_area[(i+1)%4], CV_RGB(0,255,0), 2);

	// map sharpness_area into the roi
	for (unsigned int i=0; i<sharpness_area.size(); ++i)
		sharpness_area[i] -= min_point;

	// variant M_V (std. dev. of gray values)
	std::vector<uchar> gray_values;
	double avg_gray = 0.;
	for (int v=0; v<roi.rows; ++v)
	{
		for (int u=0; u<roi.cols; ++u)
		{
			if (cv::pointPolygonTest(sharpness_area, cv::Point2i(u,v), false) > 0)
			{
				uchar val = gray_image.at<uchar>(v,u);
				gray_values.push_back(val);
				avg_gray += (double)val;
			}
		}
	}
	int pixel_count = (int)gray_values.size();
	if (pixel_count > 0)
		avg_gray /= (double)pixel_count;
	double sharpness_score = 0.;
	for (int i=0; i<pixel_count; ++i)
		sharpness_score += (double)((int)gray_values[i]-(int)avg_gray)*((int)gray_values[i]-(int)avg_gray);
//		std::cout << "pixel_count=" << pixel_count << " \t sharpness score=" << sharpness_score << std::endl;

//		double m = 9139.749632393357;	// these numbers come from measuring pixel_count and sharpness_score in all possible situations and interpolating a function (here: a linear function y=m*x+n) with that data
//		double n = -2670187.875850272;
	sharpness_measure = std::min(1., sharpness_score / (sharpness_calibration_parameter_m * pixel_count + sharpness_calibration_parameter_n));	// how far is the score from the linear sharpness function

//		std::cout << "sharpness_score_normalized=" << sharpness_score_normalized << std::endl;

	if (m_log_or_calibrate_sharpness_measurements == true)
	{
		SharpnessLogData log;
		log.distance_to_camera= cv::norm(pose_CfromO.trans, cv::NORM_L2);
		log.pixel_count = pixel_count;
		log.sharpness_score = sharpness_score;
		m_log_data.push_back(log);

		cv::imshow("image", image);
		int key = cv::waitKey(10);
		if (key == 's')
		{
			std::ofstream file("sharpness_log_file.txt", std::ios::out);
			if (file.is_open() == false)
				std::cout << "Error: AbstractFiducialModel::GetSharpnessMeasure: Could not open file." << std::endl;
			else
			{
				for (unsigned int i=0; i<m_log_data.size(); ++i)
					file << m_log_data[i].pixel_count << "\t" << m_log_data[i].distance_to_camera << "\t" << m_log_data[i].sharpness_score << "\n";
				file.close();
				std::cout << "Info: AbstractFiducialModel::GetSharpnessMeasure: All data successfully written to disk." << std::endl;
			}
		}
		else if (key == 'c')
		{
			// compute linear regression for calibration curve and output result on screen
			// i.e. the m and n parameters come from measuring pixel_count and sharpness_score in all possible situations
			// and interpolating a function (here: a linear function y=m*x+n) with that data
			cv::Mat A = cv::Mat::ones(int(m_log_data.size()), 2, CV_64FC1);
			cv::Mat b(int(m_log_data.size()), 1, CV_64FC1);
			for (unsigned int i=0; i<m_log_data.size(); ++i)
			{
				A.at<double>(i,0) = m_log_data[i].pixel_count;
				b.at<double>(i) = m_log_data[i].sharpness_score;
			}
			cv::Mat line_parameters;
			cv::solve(A, b, line_parameters, cv::DECOMP_QR);

			std::cout << "The line parameters for the sharpness measure calibration curve are:\n  m = " << std::setprecision(15) << line_parameters.at<double>(0) << "\n  n = " << line_parameters.at<double>(1) << "\n\nPress any key to record further data and calibrate again with the present the additional data.\n" << std::endl;
			cv::waitKey();
		}
	}

//		cv::imshow("sharpness area", image_copy);
//		cv::waitKey(10);

	return ipa_Utils::RET_OK;
}


cv::Mat AbstractFiducialModel::GetCameraMatrix()
{
	return m_camera_matrix;
}

unsigned long AbstractFiducialModel::SetCameraMatrix(cv::Mat camera_matrix)
{
	m_camera_matrix = camera_matrix.clone();
	return ipa_Utils::RET_OK;
}

cv::Mat AbstractFiducialModel::GetDistortionCoeffs()
{
	if (m_dist_coeffs.empty())
		return cv::Mat::zeros(1, 4, CV_64FC1);
	return m_dist_coeffs;
}

unsigned long AbstractFiducialModel::SetDistortionCoeffs(cv::Mat dist_coeffs)
{
	m_dist_coeffs = dist_coeffs.clone();
	return ipa_Utils::RET_OK;
}

AbstractFiducialParameters AbstractFiducialModel::GetGeneralFiducialParameters(int marker_id)
{
	std::map<int, AbstractFiducialParameters>::iterator it = m_general_fiducial_parameters.find(marker_id);
	if (it==m_general_fiducial_parameters.end())
	{
		AbstractFiducialParameters empty;
		empty.m_id = -1;
		return empty;
	}
	return it->second;
}
