/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2015 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: cob_object_perception
 * \note
 * ROS package name: cob_surface_classification
 *
 * \author
 * Author: Richard Bormann
 * \author
 * Supervised by:
 *
 * \date Date of creation: 25.02.2015
 *
 * \brief
 *
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

#include <string>

// ROS includes
#include <ros/ros.h>

#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/DeleteModel.h>

int main (int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "cob_surface_classification");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	std::string getStateServiceName = "/gazebo/get_world_properties";
	std::string deleteModelsServiceName = "/gazebo/delete_model";

	// here we wait until the service is available; please use the same service name as the one in the server; you may define a timeout if the service does not show up
	bool serviceAvailable = true;
	serviceAvailable &= ros::service::waitForService(getStateServiceName, 5000);
	serviceAvailable &= ros::service::waitForService(deleteModelsServiceName, 5000);

	// only proceed if the service is available
	if (serviceAvailable == false)
	{
		std::cout << "One or multiple services could not be found.\n" << std::endl;
		return -1;
	}
	std::cout << "The service servers is advertised.\n" << std::endl;

	std::vector<std::string> model_names;
	// 1. get list of objects
	{
		// prepare the request and response messages
		gazebo_msgs::GetWorldProperties::Request req;
		gazebo_msgs::GetWorldProperties::Response res;
		// this calls the service server to process our request message and put the result into the response message
		// this call is blocking, i.e. this program will not proceed until the service server sends the response
		bool success = ros::service::call(getStateServiceName, req, res);
		if (success == true)
		{
			ROS_INFO("List of objects retrieved.\n");
			model_names = res.model_names;
		}
		else
		{
			ROS_INFO("Request for retrieving the current object list failed.\n");
			return(0);
		}
	}

	// 2. delete all models except for world
	for (int i=0; i<model_names.size(); ++i)
	{
		// do not delete the world or robot
		if (model_names[i].compare("ground_plane")==0 || model_names[i].compare("robot")==0)
			continue;

		// prepare the request and response messages
		gazebo_msgs::DeleteModel::Request req;
		gazebo_msgs::DeleteModel::Response res;
		req.model_name = model_names[i];
		// this calls the service server to process our request message and put the result into the response message
		// this call is blocking, i.e. this program will not proceed until the service server sends the response
		bool success = ros::service::call(deleteModelsServiceName, req, res);
//		if (success == true)
//			ROS_INFO("Object deleted.\n");
//		else
//			ROS_INFO("Request for deleting object failed.\n");
	}
	return (0);
}
