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
* Combines all three laser scanner based leg detectors running on cob3.
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

#include "geometry_msgs/PolygonStamped.h"

#include <vector>


class LegDetectionAccumulator
{
public:

	LegDetectionAccumulator(ros::NodeHandle nh)
	: node_handle_(nh)
	{
		person_location_sub_front_ = node_handle_.subscribe<geometry_msgs::PolygonStamped>("detected_humans_laser_front", 5, &LegDetectionAccumulator::humanDetectionCallback, this);
		person_location_sub_rear_ = node_handle_.subscribe<geometry_msgs::PolygonStamped>("detected_humans_laser_rear", 5, &LegDetectionAccumulator::humanDetectionCallback, this);
		person_location_sub_top_ = node_handle_.subscribe<geometry_msgs::PolygonStamped>("detected_humans_laser_top", 5, &LegDetectionAccumulator::humanDetectionCallback, this);
	}

	void init()
	{
		// Parameters
		std::cout << "\n--------------------------\nLeg Detection Accumulator Parameters:\n--------------------------\n";
//		node_handle_.param("leg_detection/max_leg_distance", max_leg_distance_, 0.5);
//		std::cout << "max_leg_distance = " << max_leg_distance_ << std::endl;
	}

	void humanDetectionCallback(const geometry_msgs::PolygonStampedConstPtr& detection_msg)
	{

	}

private:

	struct Point2d
	{
		Point2d(double x_, double y_)
		{
			x = x_;
			y = y_;
			observation_time = ros::Time::now();
		}

		double x;
		double y;
		ros::Time observation_time;
	};

	ros::NodeHandle node_handle_;
	ros::Subscriber person_location_sub_front_;
	ros::Subscriber person_location_sub_rear_;
	ros::Subscriber person_location_sub_top_
	tf::TransformListener tf_listener_;

	std::vector<Point2d> detection_accumulator_;

	// parameters
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "leg_detection_accumulator");

	ros::NodeHandle n;

	LegDetectionAccumulator lda(n);
	lda.init();

	ros::spin();

	return 0;
}
