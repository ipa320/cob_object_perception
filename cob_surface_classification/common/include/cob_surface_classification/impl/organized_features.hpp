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
 *  		Carolin Eckard
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

#ifndef __IMPL_ORGANIZED_FEATURES_H__
#define __IMPL_ORGANIZED_FEATURES_H__

template <typename PointInT, typename PointOutT> int
cob_features::OrganizedFeatures<PointInT,PointOutT>::searchForNeighbors(
  int index,
  std::vector<int>& indices)
{
  int idx;
  int idx_x = index % surface_->width;
  int idx_y = index * inv_width_;

  std::vector<std::vector<int> >::iterator it_c;
  std::vector<int>::iterator it_ci;

  indices.clear();
  indices.reserve((int)(pixel_search_radius_ * pixel_search_radius_));

  if (idx_y >= pixel_search_radius_ && idx_y < (int)surface_->height - pixel_search_radius_ &&
      idx_x >= pixel_search_radius_ && idx_x < (int)surface_->width - pixel_search_radius_)
  {
    for (it_c = mask_.begin(); it_c != mask_.end(); it_c++) // iterate circles
    {
      for (it_ci = (*it_c).begin(); it_ci != (*it_c).end(); it_ci++) // iterate current circle
      {
        idx = index + *it_ci;
        if (pcl_isnan(surface_->points[idx].z) ) continue;
        indices.push_back(idx);
      }
    }
  }
  else
  {
    for (it_c = mask_.begin(); it_c != mask_.end(); it_c++) // iterate circles
    {
      for (it_ci = (*it_c).begin(); it_ci != (*it_c).end(); it_ci++) // iterate current circle
      {
        idx = index + *it_ci;
        if ( idx < 0 || idx >= (int)surface_->points.size() ) continue;
        int v = idx * inv_width_;
        if (v < 0 || v >= (int)surface_->height) continue;
        if (pcl_isnan(surface_->points[idx].z)) continue;
        indices.push_back(idx);
      }
    }
  }
  return 1;
}

template <typename PointInT, typename PointOutT> int
cob_features::OrganizedFeatures<PointInT,PointOutT>::searchForNeighborsInRange(
  int index,
  std::vector<int>& indices)
{
  int idx;
  int idx_x = index % surface_->width;
  int idx_y = index * inv_width_;
  const PointInT* p = &(surface_->points[index]);
  const PointInT* p_i;
  float distance_threshold = skip_distant_point_threshold_ * 0.003 * p->z * p->z;

  std::vector<std::vector<int> >::iterator it_c;
  std::vector<int>::iterator it_ci;

  indices.clear();
  indices.reserve(n_points_);

  if (idx_y >= pixel_search_radius_ && idx_y < (int)surface_->height - pixel_search_radius_ &&
      idx_x >= pixel_search_radius_ && idx_x < (int)surface_->width - pixel_search_radius_)
  {
    for (it_c = mask_.begin(); it_c != mask_.end(); it_c++) // iterate circles
    {
      for (it_ci = (*it_c).begin(); it_ci != (*it_c).end(); it_ci++) // iterate current circle
      {
        idx = index + *it_ci;
        p_i = &(surface_->points[idx]);
        if ( pcl_isnan(p_i->z) ) continue;
        if ( fabs(p_i->z - p->z) > distance_threshold ) continue;

        indices.push_back(idx);
      }
    }
  }
  else
  {
    for (it_c = mask_.begin(); it_c != mask_.end(); it_c++) // iterate circles
    {
      for (it_ci = (*it_c).begin(); it_ci != (*it_c).end(); it_ci++) // iterate current circle
      {
        idx = index + *it_ci;
        if ( idx < 0 || idx >= (int)surface_->points.size() ) continue;
        int v = idx * inv_width_;
        if (v < 0 || v >= (int)surface_->height) continue;
        p_i = &(surface_->points[idx]);
        if ( pcl_isnan(p_i->z) ) continue;
        if ( fabs(p_i->z - p->z) > distance_threshold ) continue;

        indices.push_back(idx);
      }
    }
  }
  if (indices.size() > 1)
    return 1;
  else
    return -1;
}


template <typename PointInT, typename PointOutT> int
cob_features::OrganizedFeatures<PointInT,PointOutT>::searchForNeighborsInRange(
  int index,
  std::vector<int>& indices,
  std::vector<float>& sqr_distances)
{
  int idx;
  int idx_x = index % surface_->width;
  int idx_y = index * inv_width_;
  const PointInT* p = &(surface_->points[index]);
  const PointInT* p_i;
  float distance_threshold = skip_distant_point_threshold_ * 0.003 * p->z * p->z, d_z;
  std::vector<std::vector<int> >::iterator it_c;
  std::vector<int>::iterator it_ci;

  indices.clear();
  indices.reserve(n_points_);
  sqr_distances.clear();
  sqr_distances.reserve(n_points_);

  if (idx_y >= pixel_search_radius_ && idx_y < (int)surface_->height - pixel_search_radius_ &&
      idx_x >= pixel_search_radius_ && idx_x < (int)surface_->width - pixel_search_radius_)
  {
    for (it_c = mask_.begin(); it_c != mask_.end(); it_c++) // iterate circles
    {
      for (it_ci = (*it_c).begin(); it_ci != (*it_c).end(); it_ci++) // iterate current circle
      {
        idx = index + *it_ci;
        p_i = &(surface_->points[idx]);
        if ( pcl_isnan(p_i->z) ) continue;
        if ( (d_z = fabs(p_i->z - p->z)) > distance_threshold ) continue;

        indices.push_back(idx);
        sqr_distances.push_back(pow(p_i->x - p->x, 2) + pow(p_i->y - p->y, 2) + pow(d_z, 2));
      }
    }
  }
  else
  {
    for (it_c = mask_.begin(); it_c != mask_.end(); it_c++) // iterate circles
    {
      for (it_ci = (*it_c).begin(); it_ci != (*it_c).end(); it_ci++) // iterate current circle
      {
        idx = index + *it_ci;
        if ( idx < 0 || idx >= (int)surface_->points.size() ) continue;
        int v = idx * inv_width_;
        if (v < 0 || v >= (int)surface_->height) continue;
        p_i = &(surface_->points[idx]);
        if ( pcl_isnan(p_i->z) ) continue;
        if ( (d_z = fabs(p_i->z - p->z)) > distance_threshold ) continue;

        indices.push_back(idx);
        sqr_distances.push_back(pow(p_i->x - p->x, 2) + pow(p_i->y - p->y, 2) + pow(d_z, 2));
      }
    }
  }
  if (indices.size() > 1)
    return 1;
  else
    return -1;
}

template <typename PointInT, typename PointOutT> bool
cob_features::OrganizedFeatures<PointInT,PointOutT>::initCompute()
{
  if (!pcl::PCLBase<PointInT>::initCompute ())
  {
    PCL_ERROR ("[pcl::%s::initCompute] Init failed.\n", getClassName ().c_str ());
    return (false);
  }

  // If the dataset is empty, just return
  if (input_->points.empty ())
  {
    PCL_ERROR ("[pcl::%s::compute] input_ is empty!\n", getClassName ().c_str ());
    // Cleanup
    deinitCompute ();
    return (false);
  }

  // If no search surface has been defined, use the input dataset as the search surface itself
  if (!surface_)
  {
    fake_surface_ = true;
    surface_ = input_;
  }

  // Check if input is orgnized
  if ( !(surface_->isOrganized () && input_->isOrganized ()) )
  {
    PCL_ERROR ("[pcl::%s::compute] input_ and surface_ is not organized!\n", getClassName ().c_str ());
    // Cleanup
    deinitCompute ();
    return (false);
  }

  // Do a fast check to see if the search parameters are well defined
  if (pixel_search_radius_ == 0 || pixel_steps_ == 0 || circle_steps_ == 0)
  {
    PCL_ERROR ("[pcl::%s::compute] Radius not defined! ", getClassName ().c_str ());
    // Cleanup
    deinitCompute ();
    return (false);
  }
  else
  {
    inv_width_ = 1.0f / surface_->width;
    if (mask_changed)
    {
      int num_circles = std::floor(pixel_search_radius_ / circle_steps_);
      n_points_ = pow(2 * pixel_search_radius_ + 1, 2);
      // create a new mask
      mask_.clear();
      for (int circle = 0; circle < num_circles; circle++)
      {
        int circle_size = pixel_search_radius_ - (circle*circle_steps_);
        int dy = circle_size * surface_->width;

        std::vector<int> new_circle;

        for (int x = circle_size; x >= -circle_size; x -= pixel_steps_)
          new_circle.push_back( x - dy );
        for (int y = -circle_size+pixel_steps_; y <= circle_size-pixel_steps_; y += pixel_steps_)
          new_circle.push_back( -circle_size + y * surface_->width );
        for (int x = -circle_size; x <= +circle_size; x += pixel_steps_)
          new_circle.push_back( x + dy );
        for (int y = circle_size-pixel_steps_; y >= -circle_size+pixel_steps_; y -= pixel_steps_)
          new_circle.push_back( circle_size + y * surface_->width );

        mask_.push_back(new_circle);
      }
    }
  }

  return (true);
}

template <typename PointInT, typename PointOutT> bool
cob_features::OrganizedFeatures<PointInT,PointOutT>::deinitCompute()
{
  // Reset the surface
  if (fake_surface_)
  {
    surface_.reset ();
    fake_surface_ = false;
  }
  return (true);
}

template <typename PointInT, typename PointOutT> void
cob_features::OrganizedFeatures<PointInT,PointOutT>::compute(PointCloudOut &output)
{
  if (!initCompute ())
  {
    output.width = output.height = 0;
    output.points.clear ();
    return;
  }

  // Copy the header
  output.header = input_->header;

  // Resize the output dataset
  if (output.points.size () != indices_->size ())
    output.points.resize (indices_->size ());
  // Check if the output will be computed for all points or only a subset
  if (indices_->size () != input_->points.size ())
  {
    output.width = (int) indices_->size ();
    output.height = 1;
  }
  else
  {
    output.width = input_->width;
    output.height = input_->height;
  }
  output.is_dense = input_->is_dense;

  // Perform the actual feature computation
  computeFeature (output);

  deinitCompute ();
}

template <typename PointInT, typename PointOutT> void
cob_features::OrganizedFeatures<PointInT,PointOutT>::computeMaskManually(int cloud_width)
{
  //set up circles of decreasing size (from pixel_search_radius to 0)

	int num_circles = std::floor(pixel_search_radius_ / circle_steps_);
  n_points_ = pow(2 * pixel_search_radius_ + 1, 2);
  // create a new mask
  mask_.clear();
  for (int circle = 0; circle < num_circles; circle++)
  {
    int circle_size = pixel_search_radius_ - (circle*circle_steps_);
    int dy = circle_size * cloud_width;

    std::vector<int> new_circle;

    for (int x = circle_size; x >= -circle_size; x -= pixel_steps_)
      new_circle.push_back( x - dy );
    for (int y = -circle_size+pixel_steps_; y <= circle_size-pixel_steps_; y += pixel_steps_)
      new_circle.push_back( -circle_size + y * cloud_width );
    for (int x = -circle_size; x <= +circle_size; x += pixel_steps_)
      new_circle.push_back( x + dy );
    for (int y = circle_size-pixel_steps_; y >= -circle_size+pixel_steps_; y -= pixel_steps_)
      new_circle.push_back( circle_size + y * cloud_width );

    mask_.push_back(new_circle);
  }
  mask_changed = false;
}

template <typename PointInT, typename PointOutT> void
cob_features::OrganizedFeatures<PointInT,PointOutT>::computeMaskManually_increasing(int cloud_width)
{
  //set up circles of increasing size (from pixel_search_radius to circle_steps_)

	int num_circles = std::floor(pixel_search_radius_ / circle_steps_);
  n_points_ = pow(2 * pixel_search_radius_ + 1, 2);
  // create a new mask
  mask_.clear();
  for (int circle = 1; circle <= num_circles; circle++)
  {
    int circle_size = circle*circle_steps_;
    int dy = circle_size * cloud_width;

    std::vector<int> new_circle;

    for (int x = circle_size; x >= -circle_size; x -= pixel_steps_)
      new_circle.push_back( x - dy );
    for (int y = -circle_size+pixel_steps_; y <= circle_size-pixel_steps_; y += pixel_steps_)
      new_circle.push_back( -circle_size + y * cloud_width );
    for (int x = -circle_size; x <= +circle_size; x += pixel_steps_)
      new_circle.push_back( x + dy );
    for (int y = circle_size-pixel_steps_; y >= -circle_size+pixel_steps_; y -= pixel_steps_)
      new_circle.push_back( circle_size + y * cloud_width );

    mask_.push_back(new_circle);
  }
  mask_changed = false;
}

#endif
