/*
 * PCLVisualization.cpp
 *
 *  Created on: 03.07.2013
 *      Author: matthias
 */

#include <PCLVisualization.h>

//-------------------------------------------------Constructor/Destructor------------------------------------------------------
PCLVisualization::PCLVisualization() {
	Init();
}

PCLVisualization::~PCLVisualization() {
	// TODO Auto-generated destructor stub
}


//-------------------------------------------------init()------------------------------------------------------
void PCLVisualization::Init(){
	basic_cloud_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	point_cloud_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	PCLviewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	cubecounter = 0;
	realtimeid = 0;
}


//-------------------------------------------------addVecToCloud()------------------------------------------------------
void PCLVisualization::addVecToCloud(cv::Mat vec3d,int rr,int gg ,int bb){

	uint8_t r(rr), g(gg), b(bb);
	pcl::PointXYZ basic_point;
	basic_point.x = vec3d.at<double>(0);
	basic_point.y = vec3d.at<double>(1);
	basic_point.z = vec3d.at<double>(2);

	pcl::PointXYZRGB point;
	point.x = basic_point.x;
	point.y = basic_point.y;
	point.z = basic_point.z;

	uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	point.rgb = *reinterpret_cast<float*>(&rgb);

	point_cloud_ptr->points.push_back(point);

}

void PCLVisualization::addRealtimeInformation(cv::Mat location, cv::Mat orientation1, cv::Mat orientation2, double size){

	//unsigned int pathlength = 5;
	unsigned int cubeid = 1;
	unsigned int lineid1 = 2;
	unsigned int lineid2 = 3;

	char str[512];
	if(realtimeid >= 1){
		sprintf (str, "%u", cubeid);
		PCLviewer->removeShape(str);
		sprintf (str, "%u", lineid1);
		PCLviewer->removeShape(str);
		sprintf (str, "%u", lineid2);
		PCLviewer->removeShape(str);
	}

	sprintf (str, "%u", lineid1);
    PCLviewer->addLine(pcl::PointXYZ((float)location.at<double>(0),(float)location.at<double>(1),(float)location.at<double>(2)),
    				   pcl::PointXYZ((float)orientation1.at<double>(0),(float)orientation1.at<double>(1),(float)orientation1.at<double>(2)),str);

	sprintf (str, "%u", lineid2);
    PCLviewer->addLine(pcl::PointXYZ((float)location.at<double>(0),(float)location.at<double>(1),(float)location.at<double>(2)),
    				   pcl::PointXYZ((float)orientation2.at<double>(0),(float)orientation2.at<double>(1),(float)orientation2.at<double>(2)),str);

    sprintf (str, "%u", cubeid);
	PCLviewer->addCube(Eigen::Vector3f(location.at<double>(0),location.at<double>(1),location.at<double>(2)),Eigen::Quaternionf(1,0,0,0),size,size,size,str);

	realtimeid++;
}

void PCLVisualization::spin(){
	PCLviewer->spinOnce (1,true);
}

void PCLVisualization::addCube(cv::Mat vec3d,double size){

	std::stringstream ss;
	ss << cubecounter;
	PCLviewer->addCube(Eigen::Vector3f(vec3d.at<double>(0),vec3d.at<double>(1),vec3d.at<double>(2)),Eigen::Quaternionf(1,0,0,0),size,size,size,ss.str().c_str(),0);
	cubecounter++;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> PCLVisualization::rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud){
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  PCLviewer->setBackgroundColor (20, 20, 20);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  PCLviewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  PCLviewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  PCLviewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "sample cloud");
  PCLviewer->addCoordinateSystem (100);
  PCLviewer->initCameraParameters ();
  return (PCLviewer);
}


//-------------------------------------------------setCameraToPOV()----------------------------------------------------------------------
void PCLVisualization::setCameraToPOV(double x,double y,double z,double viewX,double viewY,double viewZ,double upX,double upY,double upZ){


	//PCLviewer->setCameraPosition(camera_x,camera_y,camera_z,camera_alpha,camera_beta,camera_gamma);
	//PCLviewer->setCameraPose(camera_x,camera_y,camera_z,0,0,0,camera_alpha,camera_beta,camera_gamma);
	PCLviewer->setCameraPose(x,y,z,viewX,viewY,viewZ,upX,upY,upZ);
	//PCLviewer->
    PCLviewer->spinOnce (1,true);
    //PCLviewer->updateCamera();
}


//-------------------------------------------------showCloud()------------------------------------------------------
void PCLVisualization::showCloud(){

	  //PCLviewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&PCLviewer);
	  //PCLviewer->registerMouseCallback (mouseEventOccurred, (void*)&PCLviewer);

	  point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
	  point_cloud_ptr->height = 1;


	  rgbVis(point_cloud_ptr);

	  PCLviewer->setCameraPose(1000,1000,1000,0,0,0,0,0,0);

//	  while (!PCLviewer->wasStopped ()){
//		    //PCLviewer->setCameraPosition(camera_x,camera_y,camera_z,camera_alpha,camera_beta,camera_gamma);
//		    PCLviewer->spinOnce (10,true);
//	  }
}


