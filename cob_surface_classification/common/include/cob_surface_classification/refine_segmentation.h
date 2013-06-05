/*
 * refine_segmentation.h
 *
 *  Created on: May 29, 2013
 *      Author: rmb-ce
 */

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_segmentation/general_segmentation.h"
#include "cob_3d_segmentation/cluster_graph_structure.h"

//Eigen
#include <pcl/common/eigen.h>




namespace cob_3d_segmentation
{
template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT>
class RefineSegmentation : public GeneralSegmentation<PointT, PointLabelT>
{
public:
	typedef pcl::PointCloud<PointT> PointCloud;
	typedef typename PointCloud::Ptr PointCloudPtr;
	typedef typename PointCloud::ConstPtr PointCloudConstPtr;

	typedef pcl::PointCloud<PointNT> NormalCloud;
	typedef typename NormalCloud::Ptr NormalCloudPtr;
	typedef typename NormalCloud::ConstPtr NormalCloudConstPtr;

	typedef pcl::PointCloud<PointLabelT> LabelCloud;
	typedef typename LabelCloud::Ptr LabelCloudPtr;
	typedef typename LabelCloud::ConstPtr LabelCloudConstPtr;

	typedef typename ClusterGraphT::Ptr ClusterGraphPtr;
	typedef typename ClusterGraphT::ClusterPtr ClusterPtr;
	typedef typename ClusterGraphT::EdgePtr EdgePtr;

public:
	RefineSegmentation ()
	:max_curv_thres_(0.05)
	,min_curv_thres_(0.05)
	,curv_dir_thres_(0.7)
	{};

	~RefineSegmentation ()
	{ };

	inline void setClusterGraphInOut(ClusterGraphPtr clusters) { graph_ = clusters; }
	inline void setNormalCloudIn(NormalCloudConstPtr normals) { normals_ = normals; }
	inline void setLabelCloudInOut(LabelCloudPtr labels) { labels_ = labels; }
	virtual void setInputCloud(const PointCloudConstPtr& points) { surface_ = points; }
	inline void setCurvThres(float max_th,float min_th, float dir_th) { max_curv_thres_ = max_th; min_curv_thres_ = min_th; curv_dir_thres_ = dir_th; }
	virtual LabelCloudConstPtr getOutputCloud() { return labels_; }
	virtual bool compute()
	{
		refineUsingCurvature();
		return true;
	}


	void refineUsingCurvature();
	void printCurvature(cv::Mat& color_image);

private:
	bool similarCurvature(ClusterPtr c1, ClusterPtr c2);

	ClusterGraphPtr graph_;
	PointCloudConstPtr surface_;
	NormalCloudConstPtr normals_;
	LabelCloudPtr labels_;
	float max_curv_thres_;
	float min_curv_thres_;
	float curv_dir_thres_;


};
}
