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
* ROS stack name: cob_object_perception
* \note
* ROS package name: cob_leg_detection
*
* \author
* Author: Richard Bormann
* \author
* Supervised by:
*
* \date Date of creation: May 2014
*
* \brief
* A simple laser scanner based leg detector.
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
#include "tf/transform_listener.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PolygonStamped.h"

#include <vector>


class LegDetection
{
public:

	LegDetection(ros::NodeHandle nh)
	: node_handle_(nh)
	{
		laser_scanner_sub_ = node_handle_.subscribe<sensor_msgs::LaserScan>("scan", 1, &LegDetection::legDetectionCallback, this);
		person_location_pub_ = node_handle_.advertise<geometry_msgs::PolygonStamped>("detected_humans_laser", 1);
	}

	void init()
	{
		// Parameters
		z_gap_ = 0.10;
		leg_width_min_ = 0.10;
		leg_width_max_ = 0.25;
		min_leg_points_ = 8;
		target_frame_ = "base_link";
		max_leg_distance_ = 0.5;
		std::cout << "\n--------------------------\nLeg Detection Parameters:\n--------------------------\n";
		node_handle_.param("leg_detection/z_gap", z_gap_, 0.10);
		std::cout << "z_gap = " << z_gap_ << std::endl;
		node_handle_.param("leg_detection/leg_width_min", leg_width_min_, 0.10);
		std::cout << "leg_width_min = " << leg_width_min_ << std::endl;
		node_handle_.param("leg_detection/leg_width_max", leg_width_max_, 0.25);
		std::cout << "leg_width_max = " << leg_width_max_ << std::endl;
		node_handle_.param("leg_detection/min_leg_points", min_leg_points_, 8);
		std::cout << "min_leg_points = " << min_leg_points_ << std::endl;
		node_handle_.param("leg_detection/max_leg_distance", max_leg_distance_, 0.5);
		std::cout << "max_leg_distance = " << max_leg_distance_ << std::endl;
		node_handle_.param<std::string>("leg_detection/target_frame", target_frame_, "base_link");
		std::cout << "target_frame = " << target_frame_ << std::endl;
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

		// classify segments as leg/non-leg and publish detected humans
		geometry_msgs::PolygonStamped human_positions;
		human_positions.header = laser_scan_msg->header;
		human_positions.header.frame_id = target_frame_;
		geometry_msgs::Polygon single_legs;
		for (unsigned int i=0; i<segments.size(); ++i)
		{
			std::vector<Point2d>& seg = segments[i];

			// 1. point count
			if (seg.size() < (unsigned int)min_leg_points_)
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
			double diag = sqrt((maxBB.x-minBB.x)*(maxBB.x-minBB.x) + (maxBB.y-minBB.y)*(maxBB.y-minBB.y));
			if (diag <= leg_width_min_ || diag >= leg_width_max_)
				continue;

			// todo: 3. leg shape
//			for (unsigned int j=0; j<seg.size(); ++j)
//			{
//
//			}

			// convert to target coordinate system
			geometry_msgs::PointStamped ps_s, ps_t;
			ps_s.header = laser_scan_msg->header;
			ps_s.point.x = (minBB.x + maxBB.x)/2.0;
			ps_s.point.y = (minBB.y + maxBB.y)/2.0;
			ps_s.point.z = 0.0;
			tf_listener_.transformPoint(target_frame_, ps_s, ps_t);

			// add to detection list
			geometry_msgs::Point32 p;
			p.x = ps_t.point.x;
			p.y = ps_t.point.y;
			p.z = ps_t.point.z;
			bool found_second_leg = false;
			for (unsigned int k=0; k<single_legs.points.size(); ++k)
			{
				// only accept when two legs are found --> no phantom leg in background between two legs
				geometry_msgs::Point32& q = single_legs.points[k];
				double dist = sqrt((p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y));
				if (dist < max_leg_distance_)
				{
					q.x = (p.x+q.x)/2.0;
					q.y = (p.y+q.y)/2.0;
					q.z = (p.z+q.z)/2.0;
					found_second_leg = true;
					human_positions.polygon.points.push_back(q);
					single_legs.points.erase((single_legs.points.begin()+k));
					break;
				}
			}
			if (found_second_leg == false)
				single_legs.points.push_back(p);
		}

		for (unsigned int k=0; k<human_positions.polygon.points.size(); ++k)
			std::cout << "leg: " << human_positions.polygon.points[k].x << ", " << human_positions.polygon.points[k].y << std::endl;
		std::cout << "------------------------------------" << std::endl;

		// publish
		if (human_positions.polygon.points.size() > 0)
			person_location_pub_.publish(human_positions);
	}

private:

	ros::NodeHandle node_handle_;
	ros::Subscriber laser_scanner_sub_;
	ros::Publisher person_location_pub_;
	tf::TransformListener tf_listener_;

	// parameters
	double z_gap_;	// in [m]
	double leg_width_min_;
	double leg_width_max_;
	int min_leg_points_;
	double max_leg_distance_;	// max distance between both legs [m]
	std::string target_frame_;

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

	LegDetection ld(n);
	ld.init();

	ros::spin();

	return 0;
}
