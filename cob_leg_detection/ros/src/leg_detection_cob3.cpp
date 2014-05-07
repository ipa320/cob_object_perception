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
#include "cob_leg_detection/TrackedHumans.h"

#include <vector>

#include <boost/thread/mutex.hpp>


class LegDetectionAccumulator
{
public:

	LegDetectionAccumulator(ros::NodeHandle nh)
	: node_handle_(nh)
	{
		person_location_sub_front_ = node_handle_.subscribe<geometry_msgs::PolygonStamped>("detected_humans_laser_front", 5, &LegDetectionAccumulator::humanDetectionCallback, this);
		person_location_sub_rear_ = node_handle_.subscribe<geometry_msgs::PolygonStamped>("detected_humans_laser_rear", 5, &LegDetectionAccumulator::humanDetectionCallback, this);
		person_location_sub_top_ = node_handle_.subscribe<geometry_msgs::PolygonStamped>("detected_humans_laser_top", 5, &LegDetectionAccumulator::humanDetectionCallback, this);
		person_location_pub_ = node_handle_.advertise<cob_leg_detection::TrackedHumans>("detected_humans_laser", 1);
	}

	void init()
	{
		// Parameters
		std::cout << "\n--------------------------\nLeg Detection Accumulator Parameters:\n--------------------------\n";
		node_handle_.param("/leg_detection/same_detection_radius", same_detection_radius_, 0.8);
		std::cout << "same_detection_radius = " << same_detection_radius_ << std::endl;
		node_handle_.param("/leg_detection/maximum_detection_lifetime", maximum_detection_lifetime_, 2.0);
		std::cout << "maximum_detection_lifetime = " << maximum_detection_lifetime_ << std::endl;
	}

	void humanDetectionCallback(const geometry_msgs::PolygonStampedConstPtr& detection_msg)
	{
		boost::mutex::scoped_lock lock(mutex_detection_accumulator_);

		// delete old entries in accumulator
		for (unsigned int j=0; j<detection_accumulator_.size(); ++j)
		{
			if ((ros::Time::now()-detection_accumulator_[j].observation_time).toSec() > maximum_detection_lifetime_)
			{
				detection_accumulator_.erase(detection_accumulator_.begin()+j);
				detection_accumulator_speed_.erase(detection_accumulator_speed_.begin()+j);
				--j;
			}
		}

		// update accumulator with detections
		for (unsigned int i=0; i<detection_msg->polygon.points.size(); ++i)
		{
			// check for existence in detection accumulator
			bool already_in_accumulator = false;
			for (unsigned int j=0; j<detection_accumulator_.size(); ++j)
			{
				double dist = sqrt((detection_accumulator_[j].x-detection_msg->polygon.points[i].x)*(detection_accumulator_[j].x-detection_msg->polygon.points[i].x)+
						(detection_accumulator_[j].y-detection_msg->polygon.points[i].y)*(detection_accumulator_[j].y-detection_msg->polygon.points[i].y));
				if (dist < same_detection_radius_)
				{
					double dx = detection_msg->polygon.points[i].x - detection_accumulator_[j].x;
					double dy = detection_msg->polygon.points[i].y - detection_accumulator_[j].y;
					double dz = detection_msg->polygon.points[i].z - detection_accumulator_[j].z;
					if (dx*dx+dy*dy > 0.15*0.15)
					{
						detection_accumulator_speed_[j].x = dx;
						detection_accumulator_speed_[j].y = dy;
						detection_accumulator_speed_[j].z = dz;
						detection_accumulator_speed_[j].observation_time = ros::Time::now();
					}

					detection_accumulator_[j].x = detection_msg->polygon.points[i].x;
					detection_accumulator_[j].y = detection_msg->polygon.points[i].y;
					detection_accumulator_[j].z = detection_msg->polygon.points[i].z;
					detection_accumulator_[j].observation_time = ros::Time::now();
					already_in_accumulator = true;
					break;
				}
			}
			if (already_in_accumulator == false)
			{
				detection_accumulator_.push_back(Point3d(detection_msg->polygon.points[i].x, detection_msg->polygon.points[i].y, detection_msg->polygon.points[i].z));
				detection_accumulator_speed_.push_back(Point3d(0.0, 0.0, 0.0));
			}
		}

		// publish accumulated results
		cob_leg_detection::TrackedHumans detected_humans;
		detected_humans.trackedHumans.resize(detection_accumulator_.size());
		for (unsigned int j=0; j<detection_accumulator_.size(); ++j)
		{
			geometry_msgs::PointStamped p;
			p.header = detection_msg->header;
			p.point.x = detection_accumulator_[j].x;
			p.point.y = detection_accumulator_[j].y;
			p.point.z = detection_accumulator_[j].z;
			detected_humans.trackedHumans[j].location = p;
			geometry_msgs::Vector3Stamped v;
			v.header = detection_msg->header;
			v.vector.x = detection_accumulator_speed_[j].x;
			v.vector.y = detection_accumulator_speed_[j].y;
			v.vector.z = detection_accumulator_speed_[j].z;
			detected_humans.trackedHumans[j].speed = v;
		}
		person_location_pub_.publish(detected_humans);
	}

private:

	struct Point3d
	{
		Point3d(double x_, double y_, double z_)
		{
			x = x_;
			y = y_;
			z = z_;
			dx = 0.0;
			dy = 0.0;
			dz = 0.0;
			observation_time = ros::Time::now();
		}

		double x;
		double y;
		double z;
		double dx;
		double dy;
		double dz;
		ros::Time observation_time;
	};

	ros::NodeHandle node_handle_;
	ros::Subscriber person_location_sub_front_;
	ros::Subscriber person_location_sub_rear_;
	ros::Subscriber person_location_sub_top_;
	ros::Publisher person_location_pub_;
	tf::TransformListener tf_listener_;

	std::vector<Point3d> detection_accumulator_;
	std::vector<Point3d> detection_accumulator_speed_;
	boost::mutex mutex_detection_accumulator_;

	// parameters
	double same_detection_radius_;
	double maximum_detection_lifetime_;
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
