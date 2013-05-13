//#include "../../../../cob_object_perception_intern/windows/src/PreCompiledHeaders/StdAfx.h"
#ifdef __LINUX__
	#include "cob_fiducials/FiducialModelPi.h"
	#include "cob_fiducials/aruco/FiducialModelAruco.h"
	#include "cob_fiducials/FiducialTestingEnvironment.h"
#else
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/FiducialModelPi.h"
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/aruco/FiducialModelAruco.h"
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/FiducialTestingEnvironment.h"
#endif

#include <opencv/highgui.h>

using namespace ipa_Fiducials;

FiducialTestingEnvironment::FiducialTestingEnvironment(cv::Mat& camera_matrix)
{	
	m_camera_matrix = camera_matrix.clone();
	m_pi_tag = boost::shared_ptr<FiducialModelPi>(new FiducialModelPi());
	m_aruco_tag = boost::shared_ptr<FiducialModelAruco>(new FiducialModelAruco());
}

FiducialTestingEnvironment::~FiducialTestingEnvironment()
{
}

unsigned long FiducialTestingEnvironment::FiducialTestAruco()
{
	// ----------------------------------- Init detector -----------------------------------------
	if (m_aruco_tag->Init(m_camera_matrix, "ConfigurationFiles/objectDetectorIni.xml") & ipa_Utils::RET_FAILED)
		return ipa_Utils::RET_FAILED;

	// ----------------------------------- Load images -----------------------------------------
	std::string dataset_name = "dataset_170413";
	std::string filename_prefix = "ConfigurationFiles/fiducials/" + dataset_name + "/";

	bool load_next_image = true;
	std::vector<cv::Mat> image_vec;
	while (load_next_image)
	{
		std::stringstream filename;
		filename << filename_prefix << "tags_" << image_vec.size() << "_coloredPC_color_8U3.png";

		cv::Mat image = cv::imread(filename.str(),-1);
		if (!image.data)
			load_next_image = false;
		else
			image_vec.push_back(image);
	}

	if (image_vec.empty())
	{
		std::cerr << "\t ... [ERROR] Loading image failed from\n";
		std::cerr << "\t ... [ERROR] " << filename_prefix << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	std::cout << "\t ... [OK] Loaded " << image_vec.size() << " images" << std::endl;

	// ----------------------------------- Recognize fiducials -----------------------------------------
	for (int i=0; i<image_vec.size(); i++)
	{
		cv::Mat rot;
		cv::Mat trans;

		std::vector<t_pose> tags_vec;
		if (m_aruco_tag->GetPose(image_vec[i], tags_vec) & ipa_Utils::RET_FAILED)
			continue;

		// Render and save results
		for (int j=0; j<tags_vec.size(); j++)
		{
			RenderPose(image_vec[i], tags_vec[j].rot, tags_vec[j].trans);
			std::stringstream filename;
			filename << filename_prefix << "result_" << i << "_" << j << "_" << tags_vec[j].id << "_coloredPC_color_8U3.png";
			cv::imwrite(filename.str(), image_vec[i]);
		}
	}	

	return ipa_Utils::RET_OK;
}

unsigned long FiducialTestingEnvironment::FiducialTestPI()
{
	// ----------------------------------- Init detector -----------------------------------------
	if (m_pi_tag->Init(m_camera_matrix, "ConfigurationFiles/objectDetectorIni.xml") & ipa_Utils::RET_FAILED)
		return ipa_Utils::RET_FAILED;

	// ----------------------------------- Load images -----------------------------------------
	std::string dataset_name = "dataset_170413";
	std::string filename_prefix = "ConfigurationFiles/fiducials/" + dataset_name + "/";

	bool load_next_image = true;
	std::vector<cv::Mat> image_vec;
	while (load_next_image)
	{
		std::stringstream filename;
		filename << filename_prefix << "tags_" << image_vec.size() << "_coloredPC_color_8U3.png";

		cv::Mat image = cv::imread(filename.str(),-1);
		if (!image.data)
			load_next_image = false;
		else
			image_vec.push_back(image);
	}

	if (image_vec.empty())
	{
		std::cerr << "\t ... [ERROR] Loading image failed from\n";
		std::cerr << "\t ... [ERROR] " << filename_prefix << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	std::cout << "\t ... [OK] Loaded " << image_vec.size() << " images" << std::endl;

	// ----------------------------------- Recognize fiducials -----------------------------------------
	for (int i=0; i<image_vec.size(); i++)
	{
		cv::Mat rot;
		cv::Mat trans;

		std::vector<t_pose> tags_vec;
		if (m_pi_tag->GetPose(image_vec[i], tags_vec) & ipa_Utils::RET_FAILED)
			continue;

		// Render and save results
		for (int j=0; j<tags_vec.size(); j++)
		{
			RenderPose(image_vec[i], tags_vec[j].rot, tags_vec[j].trans);
			std::stringstream filename;
			filename << filename_prefix << "result_" << i << "_" << j << "_" << tags_vec[j].id << "_coloredPC_color_8U3.png";
			cv::imwrite(filename.str(), image_vec[i]);
		}
	}	

	return ipa_Utils::RET_OK;
}

unsigned long FiducialTestingEnvironment::RenderPose(cv::Mat& image, cv::Mat& rot_3x3_CfromO, cv::Mat& trans_3x1_CfromO)
{
	cv::Mat object_center(3, 1, CV_64FC1);
	double* p_object_center = object_center.ptr<double>(0);
	p_object_center[0] = 0;
	p_object_center[1] = 0;
	p_object_center[2] = 0;

	cv::Mat rot_inv = rot_3x3_CfromO.inv();

	// Compute coordinate axis for visualization
	cv::Mat pt_axis(4, 3, CV_64FC1);
	double* p_pt_axis = pt_axis.ptr<double>(0);
	p_pt_axis[0] = 0 + p_object_center[0];
	p_pt_axis[1] = 0 + p_object_center[1];
	p_pt_axis[2] = 0 + p_object_center[2];
	p_pt_axis = pt_axis.ptr<double>(1);
	p_pt_axis[0] = 0.1 + p_object_center[0];
	p_pt_axis[1] = 0 + p_object_center[1];
	p_pt_axis[2] = 0 + p_object_center[2];
	p_pt_axis = pt_axis.ptr<double>(2);
	p_pt_axis[0] = 0 + p_object_center[0];
	p_pt_axis[1] = 0.1 + p_object_center[1];
	p_pt_axis[2] = 0 + p_object_center[2];
	p_pt_axis = pt_axis.ptr<double>(3);
	p_pt_axis[0] = 0 + p_object_center[0];
	p_pt_axis[1] = 0 + p_object_center[1];
	p_pt_axis[2] = 0.1 + p_object_center[2];

	// Transform data points
	std::vector<cv::Point> vec_2d(4, cv::Point());
	for (int i=0; i<4; i++)
	{
		cv::Mat vec_3d = pt_axis.row(i).clone();
		vec_3d = vec_3d.t();
		vec_3d = rot_3x3_CfromO*vec_3d;
		vec_3d += trans_3x1_CfromO;
		double* p_vec_3d = vec_3d.ptr<double>(0);

		ReprojectXYZ(p_vec_3d[0], p_vec_3d[1], p_vec_3d[2],
			vec_2d[i].x , vec_2d[i].y);
	}

	// Render results
	int line_width = 1;
	cv::line(image, vec_2d[0], vec_2d[1], cv::Scalar(0, 0, 255), line_width);
	cv::line(image, vec_2d[0], vec_2d[2], cv::Scalar(0, 255, 0), line_width);
	cv::line(image, vec_2d[0], vec_2d[3], cv::Scalar(255, 0, 0), line_width);

	return ipa_Utils::RET_OK;
}

unsigned long FiducialTestingEnvironment::ReprojectXYZ(double x, double y, double z, int& u, int& v)
{
	cv::Mat XYZ(3, 1, CV_64FC1);
	cv::Mat UVW(3, 1, CV_64FC1);

	double* d_ptr = 0;
	double du = 0;
	double dv = 0;
	double dw = 0;

	x *= 1000;
	y *= 1000;
	z *= 1000;

	d_ptr = XYZ.ptr<double>(0);
	d_ptr[0] = x;
	d_ptr[1] = y;
	d_ptr[2] = z;

	UVW = m_camera_matrix * XYZ;
	
	d_ptr = UVW.ptr<double>(0);
	du = d_ptr[0];
	dv = d_ptr[1];
	dw = d_ptr[2];

	u = cvRound(du/dw);
	v = cvRound(dv/dw);

	return ipa_Utils::RET_OK;
}

