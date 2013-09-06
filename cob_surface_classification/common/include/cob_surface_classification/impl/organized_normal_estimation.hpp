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

#ifndef __IMPL_ORGANIZED_NORMAL_ESTIMATION_H__
#define __IMPL_ORGANIZED_NORMAL_ESTIMATION_H__

#define NEIGHBOURH_VIS false	//display neighbourhoods taken into account for normal estimation

template <typename PointInT, typename PointOutT, typename LabelOutT> bool
cob_features::OrganizedNormalEstimation<PointInT,PointOutT,LabelOutT>::compareCoordToEdgeCoord(
		int idx, int idx_x, int idx_y, std::vector<Eigen::Vector2f>& directionsOfEdges)
{
	//ignore points with coordinates x,y larger than coordinates of any edge point detected so far
	//-----------------------------------------------------------------------------------------------


	bool ignorePoint = false;

	int idx_inDepIm_x = idx % input_->width;
	int idx_inDepIm_y = idx * inv_width_;

	Eigen::Vector2f newDir;
	std::vector<Eigen::Vector2f>::iterator it_dirOfEd;
	Eigen::Vector2f ind_curr; //from query point to currently treated point in neighbourhood, "index coordinates"

	ind_curr(0) = idx_inDepIm_x - idx_x;
	ind_curr(1) = idx_inDepIm_y - idx_y;


	//check if p_i on edge
	if(edgeImage_.at<float>(idx_inDepIm_y,idx_inDepIm_x) == 0)
	{
		ignorePoint = true;
		newDir = ind_curr;
		directionsOfEdges.push_back(newDir);
	}
	//check if coordinates of p_i larger than coordinates of edge points -> p_i "behind" edge
	for(it_dirOfEd = directionsOfEdges.begin(); it_dirOfEd != directionsOfEdges.end(); ++it_dirOfEd)
	{

		if((ind_curr(0) > 0 && (*it_dirOfEd)(0) > 0 && ind_curr(0) > (*it_dirOfEd)(0))
				|| (ind_curr(0) < 0 && (*it_dirOfEd)(0) < 0 && ind_curr(0) < (*it_dirOfEd)(0))
				|| (ind_curr(1) > 0 && (*it_dirOfEd)(1) > 0 && ind_curr(1) > (*it_dirOfEd)(1))
				|| (ind_curr(1) < 0 && (*it_dirOfEd)(1) < 0 && ind_curr(1) < (*it_dirOfEd)(1)))
			ignorePoint = true;
	}
	return ignorePoint;
}


template <typename PointInT, typename PointOutT, typename LabelOutT> bool
cob_features::OrganizedNormalEstimation<PointInT,PointOutT,LabelOutT>::checkDirectionForEdge(
		int idx,	Eigen::Vector2f p_curr, std::vector<Eigen::Vector2f>&  directionsOfEdges)
{
	//ignore points if in direction of an edge. Edge directions detected so far are stored in "directionsOfEdges".
	//Directions are compared by computing the scalarproduct.
	//----------------------------------------------------------------------------------------------------------------

	bool ignorePoint = false;

	int idx_inDepIm_x = idx % input_->width;
	int idx_inDepIm_y = idx * inv_width_;

	std::vector<Eigen::Vector2f>::iterator it_dirOfEd;

	float scalProd;

	p_curr.normalize();

	//check if p_i on edge
	if(edgeImage_.at<float>(idx_inDepIm_y,idx_inDepIm_x) == 0)
	{
		ignorePoint = true;
		directionsOfEdges.push_back(p_curr);
		return ignorePoint;
	}

	//check if coordinates of p_i larger than coordinates of edge points -> p_i "behind" edge
	for(it_dirOfEd = directionsOfEdges.begin(); it_dirOfEd != directionsOfEdges.end(); ++it_dirOfEd)
	{
		scalProd = (*it_dirOfEd)(0) * p_curr(0) + (*it_dirOfEd)(1) * p_curr(1); // + (*it_dirOfEd)(2) * p_curr(2);
		if(scalProd > sameDirectionThres_)
		{
			//check if both vectors point the same direction (not in the opposite one)
			if(((*it_dirOfEd)(0) * p_curr(0) >= 0) && ( (*it_dirOfEd)(1) * p_curr(1) >= 0)) //&& ((*it_dirOfEd)(2) * p_curr(2) > 0))
			{
				ignorePoint = true;
				return ignorePoint;
			}
		}
	}
	return ignorePoint;
}



template <typename PointInT, typename PointOutT, typename LabelOutT> void
cob_features::OrganizedNormalEstimation<PointInT,PointOutT,LabelOutT>::computePointNormal (
		const PointCloudIn &cloud, int index,  float &n_x, float &n_y, float &n_z, int& label_out)
{
	//two vectors computed in the tangential plane: origin of vectors = query point,
	//end points are two points at the boundary of the neighbourhood.
	//the normal is the cross product of those two vectors
	//
	//input: index - index of point in input_ cloud
	//output: n_x, n_y, n_z - coordinates of normal vector
	//---------------------------------------------------------------------------------------------------------


	/*Timer timer;
	timer.start();
	for(int i=0; i<10; i++)
	{*/


	Eigen::Vector3f p = cloud.points[index].getVector3fMap();	//query point
	if (isnan(p(2)))
	{
		n_x = n_y = n_z = std::numeric_limits<float>::quiet_NaN();
		label_out = I_NAN;
		return;
	}


	int idx, max_gab, gab, init_gab, n_normals = 0;
	//indices of central point in edgeImage_:
	int idx_x = index % input_->width;
	int idx_y = index * inv_width_;

	//no normal estimation if point is directly on edge
	if(!edgeImage_.empty() && edgeImage_.at<float>(idx_y,idx_x) == 0)
	{
		n_x = n_y = n_z = std::numeric_limits<float>::quiet_NaN();
		label_out = I_EDGE;
		return;
	}

	//quantisation steps larger with increasing distance from camera
	//distance threshold for selecting neighourhood pixels is adapted accordingly
	float distance_threshold = skip_distant_point_threshold_ * 0.003 * p(2) * p(2);

	bool has_prev_point;	//true if a vector to a point in the neighbourhood has been computed before -> only then the normal can be computed

	std::vector<int> range_border_counter(mask_.size(), 0);
	Eigen::Vector3f p_curr;	//vector from query point to currently treated point in neighbourhood
	//Eigen::Vector2f ind_curr; //from query point to currently treated point in neighbourhood, "index coordinates"

	Eigen::Vector3f p_prev(0,0,0);	//vector from query point to previously treated point in neighbourhood
	Eigen::Vector3f p_first(0,0,0);
	Eigen::Vector3f n_idx(0,0,0);

	std::vector<std::vector<int> >::iterator it_c; // circle iterator
	std::vector<int>::iterator it_ci; // points in circle iterator
	std::vector<int>::iterator it_rbc = range_border_counter.begin();

	//indices of currently evaluated point in edgeImage_:
	int idx_inDepIm_x = 0;
	int idx_inDepIm_y = 0;
	std::vector<Eigen::Vector2f> directionsOfEdges;
	bool ignorePoint;


	// check where query point is and use out-of-image validation for neighbors or not
	if (idx_y >= pixel_search_radius_ && idx_y < (int)cloud.height - pixel_search_radius_ &&
			idx_x >= pixel_search_radius_ && idx_x < (int)cloud.width - pixel_search_radius_)
	{
		if(NEIGHBOURH_VIS)
		{
			controlImage.at<cv::Point3_<unsigned char> >(idx_y,idx_x) = cv::Point3_<unsigned char>(0,0,255);
			cv::circle(controlImage,cv::Point2f(idx_x,idx_y),pixel_search_radius_,CV_RGB(255,0,0),1);
		}

		//iterate over circles with decreasing radius (from pixel_search_radius to 0) -> cover entire circular neighbourhood from outside border to inside
		//compute normal for every pair of points on every circle (that is a specific distance to query point)
		for (it_c = mask_.begin(); it_c != mask_.end(); ++it_c, ++it_rbc) // iterate circles
		{

			has_prev_point = false; init_gab = gab = 0;
			//don't compute cross product, if the two tangential vectors are more than a quarter circle apart (prevent cross product of parallel vectors)
			max_gab = 0.25 * (*it_c).size(); // reset loop

			for (it_ci = (*it_c).begin(); it_ci != (*it_c).end(); ++it_ci) // iterate current circle
			{
				idx = index + *it_ci;
				Eigen::Vector3f p_i = cloud.points[idx].getVector3fMap();
				if ( p_i(2) != p_i(2) )                        { ++gab; continue; }               // count as gab point


				ignorePoint = false;

				//consider neighbourhood bounded by edges
				if(!edgeImage_.empty())
				{

					//ignorePoint = compareCoordToEdgeCoord(idx, idx_x,idx_y,directionsOfEdges);

					/*//real coordinates:
					 * Eigen::Vector3f xy3 = p_i - p;
					xy3.resize(2);
					Eigen::Vector2f xy = xy3;*/

					//image indices as coordinates:
					idx_inDepIm_x = idx % input_->width;
					idx_inDepIm_y = idx * inv_width_;
					Eigen::Vector2f xy;
					xy(0) = idx_inDepIm_x - idx_x;
					xy(1) = idx_inDepIm_y - idx_y;

					//ignore points if in direction of an edge detected so far
					ignorePoint = checkDirectionForEdge(idx, xy, directionsOfEdges);
				}

				//consider neighbourhood bounded by steps in depth
				else
				{
					//if depth distance larger than threshold, ignore point
					if ( fabs(p_i(2) - p(2)) > distance_threshold )
						ignorePoint = true;
				}

				if(ignorePoint){ ++gab; ++(*it_rbc); continue; }  // count as gab point


				if ( gab <= max_gab && has_prev_point ) // check if gab is small enough and a previous point exists
				{
					if(NEIGHBOURH_VIS)
					{
						idx_inDepIm_x = idx % input_->width;
						idx_inDepIm_y = idx * inv_width_;
						controlImage.at<cv::Point3_<unsigned char> >(idx_inDepIm_y,idx_inDepIm_x) = cv::Point3_<unsigned char>(0,255,0);
					}

					p_curr = p_i - p;
					n_idx += (p_prev.cross(p_curr)).normalized(); // compute normal of p_prev and p_curr
					++n_normals;
					p_prev = p_curr;
				}
				else // current is first point in circle or just after a gab
				{
					p_prev = p_i - p;
					if (!has_prev_point)
					{
						p_first = p_prev; // remember the first valid point in circle
						init_gab = gab; // save initial gab size
						has_prev_point = true;
					}
				}
				gab = 0; // found valid point, reset gab counter
			}

			// close current circle (last and first point) if gab is small enough
			if (gab + init_gab <= max_gab)
			{
				// compute normal of p_first and p_prev
				n_idx += (p_prev.cross(p_first)).normalized();
				++n_normals;
			}
		} // end loop of circles
	}
	//point near image boundaries:
	else
	{
		for (it_c = mask_.begin(); it_c != mask_.end(); ++it_c, ++it_rbc) // iterate circles
		{
			has_prev_point = false; gab = 0; max_gab = 0.25 * (*it_c).size(); // reset circle loop

			for (it_ci = (*it_c).begin(); it_ci != (*it_c).end(); ++it_ci) // iterate current circle
			{
				idx = index + *it_ci;
				// check top and bottom image border
				if ( idx < 0 || idx >= (int)cloud.points.size() ) { ++gab; continue; } // count as gab point
				int v = idx * inv_width_; // calculate y coordinate in image, // check left, right border
				if ( v < 0 || v >= (int)cloud.height || pcl_isnan(cloud.points[idx].z)) { ++gab; continue; } // count as gab point
				Eigen::Vector3f p_i = cloud.points[idx].getVector3fMap();

				ignorePoint = false;

				//consider neighbourhood bounded by edges
				if(!edgeImage_.empty())
				{
					//image indices as coordinates:
					idx_inDepIm_x = idx % input_->width;
					idx_inDepIm_y = idx * inv_width_;
					Eigen::Vector2f xy;
					xy(0) = idx_inDepIm_x - idx_x;
					xy(1) = idx_inDepIm_y - idx_y;
					ignorePoint = checkDirectionForEdge(idx, xy, directionsOfEdges);
				}

				//consider neighbourhood bounded by steps in depth
				else
				{
					//if depth distance larger than threshold, ignore point
					if ( fabs(p_i(2) - p(2)) > distance_threshold )
						ignorePoint = true;
				}

				if(ignorePoint){ ++gab; ++(*it_rbc); continue; }  // count as gab point


				if ( gab <= max_gab && has_prev_point) // check gab is small enough and a previous point exists
				{
					p_curr = p_i - p;
					n_idx += (p_prev.cross(p_curr)).normalized(); // compute normal of p_prev and p_curr
					++n_normals;
					p_prev = p_curr;
				}
				else // current is first point in circle or just after a gab
				{
					p_prev = p_i - p;
					if (!has_prev_point)
					{
						p_first = p_prev; // remember the first valid point in circle
						init_gab = gab; // save initial gab size
						has_prev_point = true;
					}
				}
				gab = 0; // found valid point, reset gab counter
			}

			// close current circle (last and first point) if gab is small enough
			if ( gab + init_gab <= max_gab)
			{
				// compute normal of p_first and p_prev
				n_idx += (p_prev.cross(p_first)).normalized();
				++n_normals;
			}
		} // end loop of circles
	}

	//average all computed normals (= "mittlere Normale")
	n_idx /= (float)n_normals;
	n_idx = n_idx.normalized();
	n_x = n_idx(0);
	n_y = n_idx(1);
	n_z = n_idx(2);

	/*}timer.stop();
	std::cout << timer.getElapsedTimeInMilliSec() << " ms for one normalEstimation, averaged over 10 iterations\n";*/


		}

template <typename PointInT, typename PointOutT, typename LabelOutT> void
cob_features::OrganizedNormalEstimation<PointInT,PointOutT,LabelOutT>::computeFeature (PointCloudOut &output)
{

	if(NEIGHBOURH_VIS)
	{
		controlImage = cv::Mat::ones(edgeImage_.rows,edgeImage_.cols,CV_8UC3) ;	//draw neighbourhood taken into account for computations

		for(int i= 0; i< edgeImage_.rows; i++)
			for(int j=0; j<edgeImage_.cols; j++)
				for(int c=0; c<3;c++)
				{
					if(edgeImage_.at<float>(i,j) == 0)
						controlImage.at<cv::Vec3b>(i,j)[c] = 0;
					else
						controlImage.at<cv::Vec3b>(i,j)[c] = 255;
				}
	}
	int count = 0;

	if (labels_->points.size() != input_->size())
	{
		labels_->points.resize(input_->size());
		labels_->height = input_->height;
		labels_->width = input_->width;
	}

	for (std::vector<int>::iterator it=indices_->begin(); it != indices_->end(); ++it)
	{

		if(!NEIGHBOURH_VIS || count % 660== 0)	//computations for visualization only at every 660th point
		{
			labels_->points[*it].label = I_UNDEF;
			computePointNormal(*surface_, *it, output.points[*it].normal[0], output.points[*it].normal[1], output.points[*it].normal[2], labels_->points[*it].label);
		}
		if(NEIGHBOURH_VIS) count++;

	}
	if(NEIGHBOURH_VIS)
	{
		cv::imshow("controlImage", controlImage);
		cv::waitKey(10);
	}

}

#endif
