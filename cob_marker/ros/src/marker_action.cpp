/*
 * Qr.cpp
 *
 *  Created on: 16.08.2012
 *      Author: josh
 */


// ROS includes
#include <ros/ros.h>
#include <pcl_ros/pcl_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <actionlib/server/simple_action_server.h>
#include <pcl/ros/conversions.h>
#include <pcl/common/pca.h>
#include <tf/transform_broadcaster.h>


#include <cob_object_detection_msgs/DetectObjectsAction.h>
#include <cob_object_detection_msgs/DetectObjectsActionGoal.h>
#include <cob_object_detection_msgs/DetectObjectsActionResult.h>

#include <cob_marker/general_marker.h>

class As_Node
{
protected:
  ros::NodeHandle n_;
public:
  As_Node(): n_("~") {
  }

  virtual ~As_Node() {}

  virtual void onInit()=0;

  void start() {

  }
};

class As_Nodelet : public  pcl_ros::PCLNodelet
{
protected:
  ros::NodeHandle n_;
public:
  As_Nodelet() {
  }

  virtual ~As_Nodelet() {}

  void start() {
    PCLNodelet::onInit();
    n_ = getNodeHandle();
  }
};

#define TEST

template<typename Parent>
class Qr_Node : public Parent
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::PointCloud2> MySyncPolicy;
  typedef actionlib::SimpleActionServer<cob_object_detection_msgs::DetectObjectsAction> ActionServer;

  message_filters::Subscriber<sensor_msgs::Image> visual_sub_ ;
  message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub_;
  message_filters::Synchronizer<MySyncPolicy> *sync_;

  ActionServer *as_;
  cob_object_detection_msgs::DetectObjectsResult result_;
  boost::mutex mutex_;

  ros::Publisher  detection_pub_;
  ros::Publisher test_pub_;
  
  tf::TransformBroadcaster br_;

  GeneralMarker *gm_;

  void subscribe() {
    visual_sub_.subscribe();
    point_cloud_sub_.subscribe();
  }
  void unsubscribe() {
    visual_sub_.unsubscribe();
    point_cloud_sub_.unsubscribe();
  }

  bool compPCA(pcl::PCA<PointCloud::PointType> &pca, const PointCloud &pc, const float w, const Eigen::Vector2i &o, const Eigen::Vector2f &d) {
    PointCloud tmp;
    for(int x=0; x<w; x++) {
      Eigen::Vector2i p = o + (d*x).cast<int>();
      if(pcl_isfinite(pc(p(0),p(1)).getVector3fMap().sum()))
        tmp.push_back( pc(p(0),p(1)) );
    }
    if(tmp.size()<2) {
      ROS_WARN("no valid points");
      return false;
    }
    pca.compute(tmp);
    return true;
  }

public:
  // Constructor
  Qr_Node():gm_(NULL)
  {
  }

  virtual ~Qr_Node()
  {
    delete sync_;
    delete gm_;
  }

  void onInit() {
    this->start();

    ros::NodeHandle *n=&(this->n_);

    visual_sub_.subscribe(*n,"/camera/rgb/image_color",2);
    point_cloud_sub_.subscribe(*n,"/camera/depth/points",2);
    sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(4), visual_sub_, point_cloud_sub_);
    sync_->registerCallback(boost::bind(&Qr_Node::callback_synchronizer, this, _1, _2));

#ifndef TEST
    unsubscribe();
#endif

    as_ = new ActionServer(*n, "object_detection", boost::bind(&Qr_Node::executeCB, this, _1), false);
    as_->start();
    //    detection_pub_ = n->advertise<cob_object_detection_msgs::Detection>("detec", 1);
    test_pub_ = n->advertise<std_msgs::String>("marker_callback",1);

    std::string algo_;
    if(n->getParam("algorithm",algo_))
    {
      if(algo_=="zxing")
      {
        gm_ = new Marker_Zxing();
        bool tryHarder;
        if(n->getParam("tryHarder",tryHarder))
          dynamic_cast<Marker_Zxing*>(gm_)->setTryHarder(tryHarder);
        ROS_INFO("using zxing algorithm");
      }
      else if(algo_=="dmtx")
      {
        gm_ = new Marker_DMTX();

        int dmtx_timeout_;
        n->param<int>("dmtx_timeout",dmtx_timeout_,500);
        dynamic_cast<Marker_DMTX*>(gm_)->setTimeout(dmtx_timeout_);
        ROS_INFO("using dmtx algorithm with %i ms timeout",dmtx_timeout_);
      }
    }

    if(!gm_)
      ROS_ERROR("no algorithm was selected\npossible candidates are:\n\t- zxing\n\t- dmtx\n");

  }

  void executeCB(const cob_object_detection_msgs::DetectObjectsGoalConstPtr &goal)
  {
    ros::Rate r(10);
    bool success = false;

    if(!gm_)
      return;

#ifndef TEST
    subscribe();
#endif
    ROS_INFO("Action called");
    double time_start = ros::Time::now().toSec();
    const double timeout = 20;
    while(ros::Time::now().toSec()-time_start<timeout)
    {
      mutex_.lock();
      for(size_t i=0; i<result_.object_list.detections.size(); i++)
      {
        if( result_.object_list.detections[i].label==goal->object_name.data)
        {
          ROS_INFO("Marker %s found", result_.object_list.detections[i].label.c_str());
          success = true;
          break;
        }
      }

      if(success)
        break;

      mutex_.unlock();

      if (as_->isPreemptRequested() || !ros::ok())
      {
        as_->setPreempted();
        break;
      }

      r.sleep();
    }

#ifndef TEST
    unsubscribe();
#endif

    if(success)
    {
      as_->setSucceeded(result_);
      mutex_.unlock();
    }
    else
    {
      as_->setAborted(result_);
    }
  }

  void callback_synchronizer(const sensor_msgs::ImageConstPtr& msg_image, const sensor_msgs::PointCloud2ConstPtr& msg_depth)
  {
    if(!gm_) return;
    double time_before_find = ros::Time::now().toSec();

    std::vector<GeneralMarker::SMarker> res;
    gm_->findPattern(*msg_image, res);
    ROS_DEBUG("\nfindPattern finished: runtime %f s ; %d pattern found", (ros::Time::now().toSec() - time_before_find), (int)res.size());

    PointCloud pc;
    if(res.size()>0)
      pcl::fromROSMsg(*msg_depth, pc);

    mutex_.lock();
    result_.object_list.detections.clear();
    for(size_t i=0; i<res.size(); i++)
    {
      //get 6DOF pose
      if(res[i].pts_.size()<3)
      {
        ROS_WARN("need 3 points");
        continue;
      }

      /*
       *   1---3
       *   |   |
       *   0---2
       */
      Eigen::Vector2f d1 = (res[i].pts_[1]-res[i].pts_[0]).cast<float>();
      Eigen::Vector2f d2 = (res[i].pts_[2]-res[i].pts_[0]).cast<float>();

      ROS_DEBUG("Code: %s", res[i].code_.c_str());

      ROS_DEBUG("p1: %d %d", res[i].pts_[0](0), res[i].pts_[0](1)); 
      ROS_DEBUG("p2: %d %d", res[i].pts_[1](0), res[i].pts_[1](1)); 
      ROS_DEBUG("p3: %d %d", res[i].pts_[2](0), res[i].pts_[2](1)); 
      ROS_DEBUG("p4: %d %d", res[i].pts_[3](0), res[i].pts_[3](1)); 
      
      ROS_DEBUG("d1: %f %f", d1(0), d1(1)); 
      ROS_DEBUG("d2: %f %f", d2(0), d2(1)); 

      int w1=std::max(std::abs(d1(0)),std::abs(d1(1)));
      int w2=std::max(std::abs(d2(0)),std::abs(d2(1)));
      d1/=w1;
      d2/=w2;

      pcl::PCA<PointCloud::PointType> pca1, pca2;
      if(!compPCA(pca1, pc, w1, res[i].pts_[0],d1))
        continue;
      if(!compPCA(pca2, pc, w2, res[i].pts_[0],d2))
        continue;

      int i1=0;
      if(pca1.getEigenValues()[1]>pca1.getEigenValues()[i1]) i1=1;
      if(pca1.getEigenValues()[2]>pca1.getEigenValues()[i1]) i1=2;
      int i2=0;
      if(pca2.getEigenValues()[1]>pca2.getEigenValues()[i2]) i2=1;
      if(pca2.getEigenValues()[2]>pca2.getEigenValues()[i2]) i2=2;

      if(pca1.getEigenVectors().col(i1).sum()<0)
        pca1.getEigenVectors().col(i1)*=-1;
      if(pca2.getEigenVectors().col(i2).sum()<0)
        pca2.getEigenVectors().col(i2)*=-1;

      Eigen::Vector3f m = (pca1.getMean()+pca2.getMean()).head<3>()/2;
      Eigen::Matrix3f M;
      M.col(0) = pca2.getEigenVectors().col(i2);
      M.col(1) = M.col(0).cross((Eigen::Vector3f)pca1.getEigenVectors().col(i1));
      M.col(1).normalize();
      M.col(2) = M.col(0).cross(M.col(1));

      Eigen::Quaternionf q(M);

      //TODO: please change to ROS_DEBUG
      //std::cout<<"E\n"<<pca1.getEigenVectors()<<"\n";
      //std::cout<<"E\n"<<pca2.getEigenVectors()<<"\n";
      //std::cout<<"E\n"<<pca1.getEigenValues()<<"\n";
      //std::cout<<"E\n"<<pca2.getEigenValues()<<"\n";

      //std::cout<<"M\n"<<M<<"\n";
      //std::cout<<"m\n"<<m<<"\n";

      cob_object_detection_msgs::Detection det;
      det.header = msg_depth->header;
      det.label = res[i].code_.substr(0,3);
      det.detector = gm_->getName();
      det.pose.header = msg_depth->header;
      det.pose.pose.position.x = m(0);
      det.pose.pose.position.y = m(1);
      det.pose.pose.position.z = m(2);
      det.pose.pose.orientation.w = q.w();
      det.pose.pose.orientation.x = q.x();
      det.pose.pose.orientation.y = q.y();
      det.pose.pose.orientation.z = q.z();
      result_.object_list.detections.push_back(det);


      //tf broadcaster for debuggin
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(m(0), m(1), m(2)) );
      transform.setRotation( tf::Quaternion(q.x(), q.y(), q.z(), q.w()) );
      br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "head_cam3d_link", res[i].code_.substr(0,3)));
    }
    mutex_.unlock();
  }


};

#ifdef COMPILE_NODELET

PLUGINLIB_DECLARE_CLASS(cob_3d_Qr, Qr_Node, Segmentation_Node_XYZ<As_Nodelet>, nodelet::Nodelet)

#else

int main(int argc, char **argv) {
  ros::init(argc, argv, "Qr");

  Qr_Node<As_Node> sn;
  sn.onInit();

  ros::spin();

  return 0;
}

#endif
