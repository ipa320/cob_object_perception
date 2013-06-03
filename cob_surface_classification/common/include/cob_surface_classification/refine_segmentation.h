/*
 * refine_segmentation.h
 *
 *  Created on: May 29, 2013
 *      Author: rmb-ce
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "cob_3d_mapping_common/point_types.h"
#include "cob_3d_segmentation/general_segmentation.h"
#include "cob_3d_segmentation/cluster_graph_structure.h"

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
	{};

	~RefineSegmentation ()
	{ };

	inline void setClusterGraphOut(ClusterGraphPtr clusters) { graph_ = clusters; }

	inline void setNormalCloudIn(NormalCloudConstPtr normals) { normals_ = normals; }
	inline void setLabelCloudInOut(LabelCloudPtr labels) { labels_ = labels; }
	virtual void setInputCloud(const PointCloudConstPtr& points) { surface_ = points; }
	virtual LabelCloudConstPtr getOutputCloud() { return labels_; }
	virtual bool compute()
	{
		refineUsingCurvature();
		return true;
	}


	void refineUsingCurvature();

private:
	ClusterGraphPtr graph_;
	PointCloudConstPtr surface_;
	NormalCloudConstPtr normals_;
	LabelCloudPtr labels_;
};
}
