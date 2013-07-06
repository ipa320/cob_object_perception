#include "ros/ros.h"

// services - here you have to include the header file with exactly the same name as your message in the /srv folder (the Message.h is automatically generated from your Message.srv file during compilation)
#include <cob_object_recording/object_recording.h>


void startRecording(std::string serviceName)
{
	// prepare the request and response messages
	cob_object_detection_msgs::StartObjectRecording::Request req;
	cob_object_detection_msgs::StartObjectRecording::Response res;

	std::cout << "Specify the name of the object that shall be recorded: ";
	std::cin >> req.object_label;

	// this calls the service server to process our request message and put the result into the response message
	// this call is blocking, i.e. this program will not proceed until the service server sends the response
	bool success = ros::service::call(serviceName, req, res);

	if (success == true)
		ROS_INFO("Object data recording successfully started.\n");
	else
		ROS_INFO("Starting object data recording failed.\n");
}

void stopRecording(std::string serviceName)
{
	// prepare the request and response messages
	cob_object_detection_msgs::StopObjectRecording::Request req;
	cob_object_detection_msgs::StopObjectRecording::Response res;

	req.stop_although_model_is_incomplete = false;

	// this calls the service server to process our request message and put the result into the response message
	// this call is blocking, i.e. this program will not proceed until the service server sends the response
	bool success = ros::service::call(serviceName, req, res);

	if (success == true)
	{
		if (res.recording_stopped == true)
			ROS_INFO("Object data recording successfully stopped.\n");
		else
			ROS_INFO("Did not stop object data recording because the collected data is not yet complete.\n");
	}
	else
		ROS_INFO("Request for stopping object data recording failed.\n");
}

void saveRecordedObject(std::string serviceName)
{
	// prepare the request and response messages
	cob_object_detection_msgs::SaveRecordedObject::Request req;
	cob_object_detection_msgs::SaveRecordedObject::Response res;

	req.storage_location = "";

	// this calls the service server to process our request message and put the result into the response message
	// this call is blocking, i.e. this program will not proceed until the service server sends the response
	bool success = ros::service::call(serviceName, req, res);

	if (success == true)
		ROS_INFO("Saving recorded object data successfully completed.\n");
	else
		ROS_INFO("Saving recorded object data failed.\n");
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	/**
	* The ros::init() function needs to see argc and argv so that it can perform
	* any ROS arguments and name remapping that were provided at the command line. For programmatic
	* remappings you can use a different version of init() which takes remappings
	* directly, but for most command-line programs, passing argc and argv is the easiest
	* way to do it.  The third argument to init() is the name of the node.
	*
	* You must call one of the versions of ros::init() before using any other
	* part of the ROS system.
	*/
	ros::init(argc, argv, "object_recording_client");

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle n;

	std::string startServiceName = "/object_recording/start_recording";
	std::string stopServiceName = "/object_recording/stop_recording";
	std::string saveServiceName = "/object_recording/save_recorded_object";

	// here we wait until the service is available; please use the same service name as the one in the server; you may define a timeout if the service does not show up
	std::cout << "Waiting for service servers to become available..." << std::endl;
	bool serviceAvailable = true;
	serviceAvailable &= ros::service::waitForService(startServiceName, 5000);
	serviceAvailable &= ros::service::waitForService(stopServiceName, 5000);
	serviceAvailable &= ros::service::waitForService(saveServiceName, 5000);

	// only proceed if the service is available
	if (serviceAvailable == false)
	{
		std::cout << "One or multiple services could not be found.\n" << std::endl;
		return -1;
	}
	std::cout << "The service servers is advertised.\n" << std::endl;

	char key = ' ';
	while(key != 'q')
	{
		std::cout << "Object Data Recording:\n\n 1. Start recording\n 2. Stop recording\n 3. Save results\n q. Quit\n\nChoose an option: ";
		key = getchar();

		if (key == '1')
			startRecording(startServiceName);
		else if (key == '2')
			stopRecording(stopServiceName);
		else if (key == '3')
			saveRecordedObject(saveServiceName);
	}


	return 0;
}
