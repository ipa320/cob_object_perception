// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/PointCloud2.h>
#include <cob_perception_msgs/PointCloud2Array.h>

// PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


class SegmentationNode
{
public:
	SegmentationNode(ros::NodeHandle nh)
	: node_handle_(nh)
	{
		input_pointcloud_sub_ = node_handle_.subscribe("input_pointcloud", 1, &SegmentationNode::inputCallback, this);
		output_pointcloud_pub_ = node_handle_.advertise<cob_perception_msgs::PointCloud2Array>("output_pointcloud_segments", 5);

		last_publishing_time_ = ros::Time::now();

		// Parameters
		std::cout << "\n--------------------------\nSegmentation Node Parameters:\n--------------------------\n";
		node_handle_.param("target_publishing_rate", target_publishing_rate_, 100.0);
		std::cout << "target_publishing_rate = " << target_publishing_rate_ << "\n";

		ROS_INFO("Segmentation node started.");
	}

	~SegmentationNode() {};

protected:
	/// callback for the incoming pointcloud data stream
	void inputCallback(const sensor_msgs::PointCloud2::ConstPtr& input_pointcloud_msg)
	{
		// forward incoming message with desired rate
		ros::Duration time_delay(1.0/target_publishing_rate_);
		//std::cout << "Time delay: " << time_delay.toSec() << std::endl;
		if (target_publishing_rate_!=0.0  &&  (ros::Time::now()-last_publishing_time_) > time_delay)
		{
			ROS_INFO("Segmenting data...");

			typedef pcl::PointXYZRGB PointType;
			pcl::PointCloud<PointType> input_pointcloud, temp;
			pcl::fromROSMsg(*input_pointcloud_msg, temp);

			// only keep points inside a defined volume
			for (unsigned int v=0; v<temp.height; v++)
				for (unsigned int u=0; u<temp.width; u++)
					if (fabs(temp.at(u,v).x)<0.2 && temp.at(u,v).z<1.2)
						input_pointcloud.push_back(temp.at(u,v));

			// Create the filtering object: downsample the dataset using a leaf size of 1cm
			pcl::VoxelGrid<PointType> vg;
			pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);
			vg.setInputCloud (input_pointcloud.makeShared());
			vg.setLeafSize (0.005f, 0.005f, 0.005f);
	//		vg.setLeafSize (0.02f, 0.02f, 0.02f);
			vg.filter (*cloud_filtered);
			std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;

			if (cloud_filtered->points.size() == 0)
				return;

			// Create the segmentation object for the planar model and set all the parameters
			pcl::SACSegmentation<PointType> seg;
			pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
			pcl::PointCloud<PointType>::Ptr cloud_plane (new pcl::PointCloud<PointType> ());
			pcl::PCDWriter writer;
			seg.setOptimizeCoefficients (true);
			seg.setModelType (pcl::SACMODEL_PLANE);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setMaxIterations (100);
			seg.setDistanceThreshold (0.02);

			int nr_points = (int) cloud_filtered->points.size ();
			while (cloud_filtered->points.size () > 0.2 * nr_points)
			{
				// Segment the largest planar component from the remaining cloud
				seg.setInputCloud(cloud_filtered);
				seg.segment (*inliers, *coefficients);
				if (inliers->indices.size () == 0)
				{
					std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
					break;
				}

				// Extract the planar inliers from the input cloud
				pcl::ExtractIndices<PointType> extract;
				extract.setInputCloud (cloud_filtered);
				extract.setIndices (inliers);
				extract.setNegative (false);

				// Write the planar inliers to disk
				extract.filter (*cloud_plane);
				std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

				// Remove the planar inliers, extract the rest
				extract.setNegative (true);
				extract.filter (*cloud_filtered);
			}

	//		cloud_filtered->header.stamp = input_pointcloud_msg->header.stamp;
	//		cloud_filtered->header.frame_id = input_pointcloud_msg->header.frame_id;
	//		sensor_msgs::PointCloud2 output_pointcloud_msg;
	//		pcl::toROSMsg(*cloud_filtered, output_pointcloud_msg);
	//		output_pointcloud_pub_.publish(output_pointcloud_msg);

			// Creating the KdTree object for the search method of the extraction
			pcl::KdTree<PointType>::Ptr tree (new pcl::KdTreeFLANN<PointType>);
			tree->setInputCloud (cloud_filtered);

			std::vector<pcl::PointIndices> cluster_indices;
			pcl::EuclideanClusterExtraction<PointType> ec;
			ec.setClusterTolerance (0.5); // 2cm
			ec.setMinClusterSize (50);
			ec.setMaxClusterSize (25000);
			ec.setSearchMethod (tree);
			ec.setInputCloud( cloud_filtered);
			ec.extract (cluster_indices);

			cob_perception_msgs::PointCloud2Array output_pointcloud_segments_msg;
			int j = 0;
			for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
			{
				pcl::PointCloud<PointType>::Ptr cloud_cluster (new pcl::PointCloud<PointType>);
				pcl::PointXYZ avgPoint;
				avgPoint.x = 0; avgPoint.y = 0; avgPoint.z = 0;
				for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
				{
					cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
					avgPoint.x += cloud_filtered->points[*pit].x;
					avgPoint.y += cloud_filtered->points[*pit].y;
				}

				std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

				if ((fabs(avgPoint.x) < cloud_cluster->points.size()*/*0.15*/0.5) && (fabs(avgPoint.y) < /*0.30*/0.5*cloud_cluster->points.size()) && (fabs(avgPoint.z) < 1.0*cloud_cluster->points.size()))
				{
					std::cout << "found a cluster in the center" << std::endl;
					cloud_cluster->header.stamp = input_pointcloud_msg->header.stamp;
					cloud_cluster->header.frame_id = input_pointcloud_msg->header.frame_id;
					sensor_msgs::PointCloud2 output_pointcloud_msg;
					pcl::toROSMsg(*cloud_cluster, output_pointcloud_msg);
					output_pointcloud_segments_msg.segments.push_back(output_pointcloud_msg);
				}
				//std::stringstream ss;
				//ss << "cloud_cluster_" << j << ".pcd";
				//writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);
				j++;
			}
			output_pointcloud_segments_msg.header = input_pointcloud_msg->header;
			output_pointcloud_pub_.publish(output_pointcloud_segments_msg);
			last_publishing_time_ = ros::Time::now();
		}
	}

	ros::Subscriber input_pointcloud_sub_;	///< incoming point cloud topic
	ros::Publisher output_pointcloud_pub_;	///< pointcloud with one segmented object

	ros::NodeHandle node_handle_;			///< ROS node handle

	// parameters
	double target_publishing_rate_;		///< rate at which the input messages are published (in Hz)
	ros::Time last_publishing_time_;	///< time of the last publishing activity

};


int main (int argc, char** argv)
{
	// Initialize ROS, spezify name of node
	ros::init(argc, argv, "object_segmentation");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create and initialize an instance of CameraDriver
	SegmentationNode segmentationNode(nh);

	ros::spin();

	return (0);
}
