/*
 *  Shows (colored) PCD-file in PCLVisualizer
 */

#define VTK_EXCLUDE_STRSTREAM_HEADERS //removes deprecated headers warning
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/condition.hpp>

int main(int argc, char* argv[])
{
  if (argc < 2)
  {
    std::cout << "Usage: showPCD [*.pcd]" << std::endl;
    return -1;
  }

  std::string pcd_file = argv[1];

  // Load the file
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr readCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_file, *readCloud) == -1)
  {
    std::cout << "PCD-File could not be loaded." << std::endl;
    return -1;
  }

  // Calculate smallest z-coordinate
  pcl::PointXYZRGB * p = new pcl::PointXYZRGB();
  p->x = 10.0;
  p->y = 10.0;
  p->z = 10.0;
  if (readCloud->points.size() > 0)
    for (unsigned int idx = 0; idx < readCloud->points.size(); ++idx)
      if (!pcl_isnan(readCloud->points[idx].x))
        if (readCloud->points[idx].z < p->z)
        {
          p->x = readCloud->points[idx].x;
          p->y = readCloud->points[idx].y;
          p->z = readCloud->points[idx].z;
        }
  std::stringstream ss;
  std::string s;
  ss << "Point_with_smallest_z:[" << p->x << "___" << p->y << "___" << p->z << "]";
  ss >> s;

  // Visualize cloud
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> clrhndlr(readCloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (readCloud, clrhndlr, "sample cloud");

  viewer->addText(s, 100, 0, "", 0);
  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }

  return 0;
}
