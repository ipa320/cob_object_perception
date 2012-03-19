#include <read_text/text_detect.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <pr2_mechanism_controllers/BaseOdometryState.h>

#include <boost/thread/condition.hpp>

using namespace std;

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class TextReader
{
	public:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	ros::Subscriber robot_state_sub_;
	DetectText detector;
	ros::Time last_movement_;

	pthread_mutex_t pr2_velocity_lock_;
	pthread_mutex_t pr2_image_lock_;

	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImagePtr detection_ptr;

	float x_;
	float y_;
	bool okToDetect_;
	bool initialized_;


	TextReader(const char* correlation, 
			const char* dictionary)
		: it_(nh_)
	{
		image_pub_ = it_.advertise("text_detect", 1);
		image_sub_ = it_.subscribe("image_color", 1, &TextReader::imageCb, this);
		robot_state_sub_ = nh_.subscribe("/base_odometry/state", 1, &TextReader::robotStateCb, this);
		detector = DetectText();
		detector.readLetterCorrelation(correlation);
		detector.readWordList(dictionary);

		pthread_mutex_init(&pr2_velocity_lock_, NULL);
		pthread_mutex_init(&pr2_image_lock_, NULL);
		okToDetect_ = false;
		initialized_ = false;

		x_ = 0;
		y_ = 0;
	}

	~TextReader()
	{}

	void robotStateCb(const pr2_mechanism_controllers::BaseOdometryStateConstPtr& msg)
	{
		pthread_mutex_trylock(&pr2_velocity_lock_);
		x_ = msg->velocity.linear.x;
		y_ = msg->velocity.linear.y;
		if (x_ != 0 || y_ != 0)
			last_movement_ = ros::Time::now();
		pthread_mutex_unlock(&pr2_velocity_lock_);
	}

	// call back function 
	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		try
		{
			pthread_mutex_trylock(&pr2_image_lock_);
			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
			if (initialized_ == false)
			{				
				detection_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
				initialized_ = true;
			}
			okToDetect_ = true;
			pthread_mutex_unlock(&pr2_image_lock_);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			okToDetect_ = false;
			return;
		}
	}
};

int main(int argc, char** argv)
{
	if (argc < 3)
	{
		ROS_ERROR( "not enought input: read_text <correlation> <dictionary>");
		return 1;
	}
	ros::init(argc, argv, "pr2_read_text");
	TextReader reader(argv[1], argv[2]);
	DetectText &detector = reader.detector;
	ros::Rate r(1);
	ros::Time last_detection = ros::Time::now();
	int32_t sys_ret;
	while (ros::ok())
	{
		if (reader.okToDetect_)
		{
		ROS_INFO("start detection........");
		ros::Time now = ros::Time::now();
		if (now - last_detection < ros::Duration(2))
		{
			ROS_INFO("waiting...");
			ros::spinOnce();
			r.sleep();
			continue;	
		}
		pthread_mutex_lock(&(reader.pr2_velocity_lock_));
		bool is_steady = (now - reader.last_movement_ > ros::Duration(2));
		bool is_moving = (reader.x_ != 0 || reader.y_ != 0);
		pthread_mutex_unlock(&(reader.pr2_velocity_lock_));
		if (!is_steady || is_moving)
		{
			ros::spinOnce();
			r.sleep();
			continue;
		}

		// do the detection
		pthread_mutex_lock(&(reader.pr2_image_lock_));
		detector.detect(reader.cv_ptr->image);
		pthread_mutex_unlock(&(reader.pr2_image_lock_));

		// publish the detection
		reader.detection_ptr->image = detector.getDetection(); 
		reader.image_pub_.publish(reader.detection_ptr->toImageMsg());
		
		int count = 0;
		for (size_t i = 0; i < detector.getWords().size(); i++)
		{
			ros::spinOnce();
			count++;
			pthread_mutex_lock(&(reader.pr2_velocity_lock_));
			is_moving = (reader.x_ != 0 || reader.y_ != 0);
			pthread_mutex_unlock(&(reader.pr2_velocity_lock_));
			if (is_moving)
			{
				break;
			}

			string word = detector.getWords()[i];
			if(word.compare("")!=0)
			{
				string command = "echo \"" + word  + "\" | festival --tts";
				sys_ret = system(command.c_str());

			}
		}
		}
		last_detection = ros::Time::now();
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
