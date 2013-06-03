/*
 * refine_segmentation.hpp
 *
 *  Created on: May 29, 2013
 *      Author: rmb-ce
 */

#ifndef REFINE_SEGMENTATION_HPP_
#define REFINE_SEGMENTATION_HPP_

#include "cob_surface_classification/refine_segmentation.h"

template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT> void
cob_3d_segmentation::RefineSegmentation<ClusterGraphT,PointT,PointNT,PointLabelT>::refineUsingCurvature()
{
	/*input: ClusterHandler, ClusterGraph
	 * output:ClusterHandler (refined clusters)
	 * */


	//DepthClusterHandler.computeCurvature();
}

#endif /* REFINE_SEGMENTATION_HPP_ */
