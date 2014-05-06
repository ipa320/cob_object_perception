/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2013 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: care-o-bot
* \note
* ROS stack name: cob_scenario_states
* \note
* ROS package name: cob_generic_states_experimental
*
* \author
* Author: Richard Bormann
* \author
* Supervised by:
*
* \date Date of creation: August 2013
*
* \brief
* cob_map_accessibility_analysis receives the map from navigation as well as obstacles and inflates_obstacles topics to assemble a common obstacle map. Upon request, this node checks the accessibility of poses within thin map by (i) checking whether the pose itself is free and by (ii) checking whether there is a closed path from robot to the goal pose.
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
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


#include "ros/ros.h"

#include "tf/tf.h"

#include "sensor_msgs/LaserScan.h"

#include <vector>


class LegDetection
{
public:

	LegDetection(ros::NodeHandle nh)
	: node_handle_(nh)
	{
		laser_scanner_sub_ = node_handle_.subscribe<sensor_msgs::LaserScan>("scan", 1, &LegDetection::legDetectionCallback, this);
		//person_location_pub_ = node_handle_.advertise<geometry_msgs::>("detected_humans_laser", 1);
	}

	void init()
	{
		// Parameters
		z_gap_ = 0.10;
		leg_width_min_ = 0.07;
		leg_width_max_ = 0.2;
		min_leg_points_ = 5;
//		std::cout << "\n--------------------------\Leg Detection Parameters:\n--------------------------\n";
//		node_handle_.param("dirt_detection/spectralResidualGaussianBlurIterations", spectralResidualGaussianBlurIterations_, 2);
//		std::cout << "spectralResidualGaussianBlurIterations = " << spectralResidualGaussianBlurIterations_ << std::endl;
	}

	void legDetectionCallback(const sensor_msgs::LaserScanConstPtr& laser_scan_msg)
	{
		// divide laser scan into single segments (by z-distance thresholding)
		std::vector<std::vector<Point2d> > segments;
		std::vector<Point2d> segment;
		for (unsigned int i=0; i<laser_scan_msg->ranges.size(); ++i)
		{
			double angle = laser_scan_msg->angle_min + i*laser_scan_msg->angle_increment; //[rad]
			double z = laser_scan_msg->ranges[i];
			double y = z*sin(angle);
			double x = z*cos(angle);

			if (i!=0 && fabs(laser_scan_msg->ranges[i]-laser_scan_msg->ranges[i-1])>z_gap_)
			{
				segments.push_back(segment);
				segment.clear();
			}
			segment.push_back(Point2d(x,y));
		}

		// classify segments as leg/non-leg
		for (unsigned int i=0; i<segments.size(); ++i)
		{
			std::vector<Point2d>& seg = segments[i];

			// 1. point count
			if (seg.size() < min_leg_points_)
				continue;

			// 2. bounding box
			Point2d minBB(1e10, 1e10);
			Point2d maxBB(-1e10, -1e10);
			for (unsigned int j=0; j<seg.size(); ++j)
			{
				if (seg[j].x < minBB.x)
					minBB.x = seg[j].x;
				if (seg[j].y < minBB.y)
					minBB.y = seg[j].y;
				if (seg[j].x > maxBB.x)
					maxBB.x = seg[j].x;
				if (seg[j].y > maxBB.y)
					maxBB.y = seg[j].y;
			}
			double diag2 = (maxBB.x-minBB.x)*(maxBB.x-minBB.x) + (maxBB.y-minBB.y)*(maxBB.y-minBB.y);
			if (diag2 <= leg_width_min_*leg_width_min_ || diag2 >= leg_width_max_*leg_width_max_)
				continue;

			// 3. leg shape

		}
	}

private:

	ros::NodeHandle node_handle_;
	ros::Subscriber laser_scanner_sub_;
	ros::Publisher person_location_pub_;

	// parameters
	double z_gap_;	// in [m]
	double leg_width_min_;
	double leg_width_max_;
	int min_leg_points_;

	struct Point2d
	{
		Point2d(double x_, double y_)
		{
			x = x_;
			y = y_;
		}

		double x;
		double y;
	};
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "leg_detection");

	ros::NodeHandle n;


	return 0;
}
