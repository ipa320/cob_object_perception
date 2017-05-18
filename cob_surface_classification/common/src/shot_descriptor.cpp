#include <cob_surface_classification/shot_descriptor.h>



void ShotHistogramDescriptor::computeShotDescriptors(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input_cloud, const pcl::PointCloud<pcl::Normal>::ConstPtr& normals, const pcl::IndicesConstPtr& indices, pcl::PointCloud<pcl::SHOT352>::Ptr& shot_descriptors)
{
	// Setup the SHOT features
	typedef pcl::SHOT352 ShotFeature;
	pcl::SHOTEstimation<pcl::PointXYZRGB, pcl::Normal, ShotFeature> shotEstimation;
	shotEstimation.setInputCloud(input_cloud);
	shotEstimation.setInputNormals(normals);
	shotEstimation.setIndices(indices);

	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>::Ptr org_search(new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>);
	shotEstimation.setSearchMethod(org_search);
	//pcl::PointCloud<ShotFeature>::Ptr shotFeatures(new pcl::PointCloud<ShotFeature>);
	//shotEstimation.setKSearch(10);
	shotEstimation.setRadiusSearch(0.05);

	// Actually compute the spin images
	shotEstimation.compute(*shot_descriptors);
	std::cout << "SHOT output points.size (): " << shot_descriptors->points.size() << std::endl;

	// Display and retrieve the SHOT descriptor for the first point.
	ShotFeature descriptor = shot_descriptors->points[0];
	std::cout << descriptor << std::endl;
}
