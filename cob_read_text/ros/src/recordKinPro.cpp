#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
//#include <cob_sensor_fusion/GetPointCloud2.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <stack>
#include <cstdio>
#include <dirent.h>

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/fstream.hpp"
#include <boost/thread/condition.hpp>

#define VTK_EXCLUDE_STRSTREAM_HEADERS //removes deprecated headers warning
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>

// DEFAULT TOPICS
#define DEFAULT_KINECT_IMAGE_TOPIC "/cam3d/rgb/image_color"     // Care-O-Bot: "/camera/rgb/image_color"
#define DEFAULT_KINECT_POINTS_TOPIC "/cam3d/rgb/points"         // Care-O-Bot: "/cam3d/rgb/points"
#define DEFAULT_PROSILICA_LEFT_TOPIC "/stereo/left/image_color"
#define DEFAULT_PROSILICA_RIGHT_TOPIC "/stereo/right/image_color"

class KinectScreenshot
{
public:
  KinectScreenshot(std::string topic1, std::string topic2, std::string topic3, std::string topic4) :
    cv_ptr(new cv_bridge::CvImage), cv_pro_R_ptr(new cv_bridge::CvImage), cv_pro_L_ptr(new cv_bridge::CvImage),
        cloud(new pcl::PointCloud<pcl::PointXYZRGB>)
  {
    std::cout << std::endl;
    std::cout << "Subscribing to topics:" << std::endl;
    std::cout << "Kinect color image: " << topic1 << std::endl;
    std::cout << "Kinect depth points: " << topic2 << std::endl;
    std::cout << "Stereo Left: " << topic3 << std::endl;
    std::cout << "Stereo right: " << topic4 << std::endl << std::endl;
    sub = n.subscribe(topic1, 1, &KinectScreenshot::ColorCb, this); // high resolution color image
    sub2 = n.subscribe(topic2, 1, &KinectScreenshot::DepthCb, this); // pointcloud
    sub3 = n.subscribe(topic3, 1, &KinectScreenshot::ProsilicaLCb, this);
    sub4 = n.subscribe(topic4, 1, &KinectScreenshot::ProsilicaRCb, this);
   // service_client_colored_pc_ = n.serviceClient<cob_sensor_fusion::GetPointCloud2>("get_colored_pc");

  }
  ~KinectScreenshot()
  {
  }

  // Callback functions
  void ColorCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  void DepthCb(sensor_msgs::PointCloud2::ConstPtr recent_image)
  {
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*recent_image, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, *cloud);
//    pcl::fromROSMsg(*recent_image, *cloud);
  }
  void ProsilicaLCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_pro_L_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  void ProsilicaRCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_pro_R_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }

  // Getters
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > getPc()
  {
    return cloud;
  }
  cv_bridge::CvImagePtr getImage()
  {
    return cv_ptr;
  }
  cv_bridge::CvImagePtr getLeftImage()
  {
    return cv_pro_L_ptr;
  }
  cv_bridge::CvImagePtr getRightImage()
  {
    return cv_pro_R_ptr;
  }

private:
  cv_bridge::CvImagePtr cv_ptr, cv_pro_R_ptr, cv_pro_L_ptr;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  ros::NodeHandle n;
  ros::Subscriber sub, sub2, sub3, sub4;
  ros::ServiceClient service_client_colored_pc_;

};

int main(int argc, char* argv[])
{
  std::string topic1(DEFAULT_KINECT_IMAGE_TOPIC), topic2(DEFAULT_KINECT_POINTS_TOPIC),
              topic3(DEFAULT_PROSILICA_LEFT_TOPIC), topic4(DEFAULT_PROSILICA_RIGHT_TOPIC);

  // Change topics via input parameters
  if (argc > 1)
  {
    // Show help
    if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "h") == 0 || strcmp(argv[1], "help") == 0 || argc > 5)
    {
      if (argc > 5)
        std::cout << "Error: Too many parameters." << std::endl;
      std::cout << std::endl;
      std::cout << "Record Kinect & Prosilica" << std::endl << std::endl;
      std::cout << "record_kinect_prosilica [topic1] [topic2] [topic3] [topic4]" << std::endl;
      std::cout << "topic1: Kinect color image.    | Default: " << DEFAULT_KINECT_IMAGE_TOPIC << std::endl;
      std::cout << "topic2: Kinect depth points.   | Default: " << DEFAULT_KINECT_POINTS_TOPIC << std::endl;
      std::cout << "topic3: Prosilica left image.  | Default: " << DEFAULT_PROSILICA_LEFT_TOPIC << std::endl;
      std::cout << "topic4: Prosilica right image. | Default: " << DEFAULT_PROSILICA_RIGHT_TOPIC << std::endl;
      return 0;
    }

    // Overwrite topic names
    switch (argc)
    {
      case 2:
      {
        topic1 = argv[1];
        break;
      }
      case 3:
      {
        topic1 = argv[1];
        topic2 = argv[2];
        break;
      }
      case 4:
      {
        topic1 = argv[1];
        topic2 = argv[2];
        topic3 = argv[3];
        break;
      }
      case 5:
      {
        topic1 = argv[1];
        topic2 = argv[2];
        topic3 = argv[3];
        topic4 = argv[4];
      }
    }
  }

  ros::init(argc, argv, "recordKinPro");

  // Set Kinect resolution to 1280x1024
  std::string cmd = "rosrun dynamic_reconfigure dynparam set /cam3d/openni_node1 image_mode 1";
  if (system(cmd.c_str()))
   {

   }

  KinectScreenshot kinectscreenshot(topic1, topic2, topic3, topic4);
  ros::Rate r(1);

  cv::Mat screenshot, pro_left, pro_right;

  // Get kinect color image

  std::cout << "Recording Kinect color image..." << std::endl;

    while (screenshot.rows == 0)
    {
      ros::spinOnce();
      r.sleep();
      screenshot = kinectscreenshot.getImage()->image;
    }

  std::cout << "Recording Left stereo image..." << std::endl;
  while (pro_left.rows == 0)
  {
    ros::spinOnce();
    r.sleep();
    pro_left = kinectscreenshot.getLeftImage()->image;
  }

  std::cout << "Recording Right stereo image..." << std::endl;
  while (pro_right.rows == 0)
  {
    ros::spinOnce();
    r.sleep();
    pro_right = kinectscreenshot.getRightImage()->image;
  }

  std::cout << "Recording Kinect depth image..." << std::endl;

    while ((kinectscreenshot.getPc())->size() == 0)
    {
      ros::spinOnce();
      r.sleep();
    }

  std::cout << "Saving all files..." << std::endl;
  // Saving in folder "/images"
  boost::filesystem::path full_path(boost::filesystem::current_path());
  std::string path = full_path.string();
  path.append("/images/safetyCopy");
  if (!boost::filesystem::exists(path))
  {
    cmd = "mkdir ";
    cmd.append(path);
    if (system(cmd.c_str()))
    {

    }
  }

  // enumerative filename: 0.png, 1.png ...
  std::vector<std::string> allFileNames;
  boost::filesystem::directory_iterator end_iter;
  for (boost::filesystem::directory_iterator dir_iter(path); dir_iter != end_iter; ++dir_iter)
    if (boost::filesystem::is_regular_file(dir_iter->status()))
#if !defined(BOOST_FILESYSTEM_VERSION) || BOOST_FILESYSTEM_VERSION<3
      allFileNames.push_back(dir_iter->filename());
#else
  	  allFileNames.push_back(dir_iter->path().filename().string());
#endif

  std::stringstream ss;
  std::string s;
  ss << allFileNames.size() / 4;
  ss >> s;

  // save kinect color image as png and kinect depth image as pcd
  cv::imwrite(path + "/" + s + ".png", screenshot);
  cv::imwrite(path + "/" + s + "l.png", pro_left);
  cv::imwrite(path + "/" + s + "r.png", pro_right);
  pcl::io::savePCDFileBinary(path + "/" + s + ".pcd", *kinectscreenshot.getPc());

  return 0;
}
