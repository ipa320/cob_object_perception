/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2012 \n
 *   Fraunhofer Institute for Manufacturing Engineering
 *   and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 *  Project name: care-o-bot
 * \note
 *  ROS stack name: cob_environment_perception_intern
 * \note
 *  ROS package name: cob_3d_mapping_features
 *
 * \author
 *  Author: Steffen Fuchs, email:georg.arbeiter@ipa.fhg.de
 * \author
 *  Supervised by: Georg Arbeiter, email:georg.arbeiter@ipa.fhg.de
 *
 * \date Date of creation: 12/2011
 *
 * \brief
 * Description:
 *
 * ToDo:
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     - Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer. \n
 *     - Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution. \n
 *     - Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef __ORGANIZED_NORMAL_ESTIMATION_H__
#define __ORGANIZED_NORMAL_ESTIMATION_H__

#include "cob_surface_classification/organized_features.h"
#include "cob_3d_mapping_common/label_defines.h"

namespace cob_features
{
  template <typename PointInT, typename PointOutT, typename LabelOutT>
    class OrganizedNormalEstimation : public OrganizedFeatures<PointInT,PointOutT>
  {
    public:

    using OrganizedFeatures<PointInT,PointOutT>::pixel_search_radius_;
    using OrganizedFeatures<PointInT,PointOutT>::pixel_steps_;
    using OrganizedFeatures<PointInT,PointOutT>::circle_steps_;
    using OrganizedFeatures<PointInT,PointOutT>::inv_width_;
    using OrganizedFeatures<PointInT,PointOutT>::mask_;
    using OrganizedFeatures<PointInT,PointOutT>::input_;
    using OrganizedFeatures<PointInT,PointOutT>::indices_;
    using OrganizedFeatures<PointInT,PointOutT>::surface_;
    using OrganizedFeatures<PointInT,PointOutT>::skip_distant_point_threshold_;
    using OrganizedFeatures<PointInT,PointOutT>::feature_name_;

    typedef pcl::PointCloud<PointInT> PointCloudIn;
    typedef typename PointCloudIn::Ptr PointCloudInPtr;
    typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

    typedef pcl::PointCloud<PointOutT> PointCloudOut;
    typedef typename PointCloudOut::Ptr PointCloudOutPtr;
    typedef typename PointCloudOut::ConstPtr PointCloudOutConstPtr;

    typedef pcl::PointCloud<LabelOutT> LabelCloudOut;
    typedef typename LabelCloudOut::Ptr LabelCloudOutPtr;
    typedef typename LabelCloudOut::ConstPtr LabelCloudOutConstPtr;

    OrganizedNormalEstimation ()
    {
      feature_name_ = "OrganizedNormalEstimation";
      sameDirectionThres_ = 0.96;
    };


    inline void
      setEdgeImage(cv::Mat& eIm)
    {
      edgeImage_ = eIm;
    }

    inline void
    setOutputLabels(LabelCloudOutPtr labels) { labels_ = labels; }

    inline void
      setSameDirectionThres(float th)
    {
      sameDirectionThres_ = th;
    }

    void computePointNormal(const PointCloudIn &cloud, int index, float &n_x, float &n_y, float &n_z, int& label_out);



    protected:

    void computeFeature (PointCloudOut &output);


    cv::Mat edgeImage_;
    float sameDirectionThres_;	//threshold for scalarproduct, so that vectors are detected as pointing in the same direction
    cv::Mat controlImage;
    LabelCloudOutPtr labels_;

    private:
    bool compareCoordToEdgeCoord(int idx, int inx_x, int inx_y, std::vector<Eigen::Vector2f>& directionsOfEdges);
    bool checkDirectionForEdge(int idx,	Eigen::Vector2f p_curr, std::vector<Eigen::Vector2f>&  directionsOfEdges);



  };
}

#include "cob_surface_classification/impl/organized_normal_estimation.hpp"

#endif
