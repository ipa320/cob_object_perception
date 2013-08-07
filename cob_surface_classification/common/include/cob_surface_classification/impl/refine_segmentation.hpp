/*
 * refine_segmentation.hpp
 *
 *  Created on: May 29, 2013
 *      Author: rmb-ce
 */

#ifndef REFINE_SEGMENTATION_HPP_
#define REFINE_SEGMENTATION_HPP_

#include "cob_surface_classification/refine_segmentation.h"


template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT> void
cob_3d_segmentation::RefineSegmentation<ClusterGraphT,PointT,PointNT,PointLabelT>::printCurvature(cv::Mat& color_image)
{
	graph_->clusters()->addBorderIndicesToClusters();
	graph_->clusters()->sortBySize(); // Ascending order
	ClusterPtr c;
	//for(ClusterPtr::iterator it_c = graph_->clusters()->begin(); it_c != graph_->clusters()->end(); ++it_c)

	//c = graph_->clusters()->getCluster(graph_->clusters()->numClusters()/2);//(int)(surface_->points.size()/2));

	c = graph_->clusters()->end();	//biggest cluster
for(int i=0;i<5;i++)
{
	if(c != graph_->clusters()->begin())
		c = --c;
}


	graph_->clusters()->computeCurvature(c);
	std::cout << "max_curv: " << c->max_curvature << std::endl;
	std::cout << "min_curv: "<< c->min_curvature << std::endl;
	std::cout << "min_curv_dir:\n "<< c->min_curvature_direction << std::endl;

   // std::vector<int> c_indices (c->indices_);
	//Eigen::Vector3f  center (c->getCentroid());
	std::vector<int>::iterator it_indices;
	//visualization
	for(it_indices = c->begin(); it_indices != c->end(); ++it_indices)
	{
		int idx_x = (int)(*it_indices % surface_->width);
		int idx_y = (int)(*it_indices / surface_->width);
		color_image.at<cv::Point3_<unsigned char> >(idx_y,idx_x) = cv::Point3_<unsigned char>(255,0,0);
	}


	cv::imshow("image", color_image);
	cv::waitKey(10);

}


template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT> bool
cob_3d_segmentation::RefineSegmentation<ClusterGraphT,PointT,PointNT,PointLabelT>::similarCurvature(ClusterPtr c1, ClusterPtr c2)
{
	//eventuell auch Verhältnis der Krümmungen vergleichen

	/*if((c1->max_curvature - c2->max_curvature) < max_curv_thres_ &&
			(c1->min_curvature - c2->min_curvature) < min_curv_thres_ &&
			(c1->min_curvature_direction.dot( c2->min_curvature_direction)) > curv_dir_thres_)	//magnitude of scalar product near 1*/
	if((c1->max_curvature - c2->max_curvature) < c1->max_curvature*0.01 &&
				(c1->min_curvature - c2->min_curvature) < c1->min_curvature*0.01 &&
				(std::abs(c1->min_curvature_direction.dot( c2->min_curvature_direction))) > curv_dir_thres_)	//Betrag!!
		{
		/*std::cout <<"similar curvature!\n";
	    std::cout << "max_curv1: " << c1->max_curvature << std::endl;
	    std::cout << "max_curv2: " << c2->max_curvature << std::endl;
	    std::cout << "min_curv1: "<< c1->min_curvature << std::endl;
	    std::cout << "min_curv2: "<< c2->min_curvature << std::endl;
	    std::cout << "min_curv_dir1:\n "<< c1->min_curvature_direction << std::endl;
	    std::cout << "min_curv_dir2:\n "<< c2->min_curvature_direction << std::endl;*/
		return true;
		}
	else
		return false;
}



template <typename ClusterGraphT, typename PointT, typename PointNT, typename PointLabelT> void
cob_3d_segmentation::RefineSegmentation<ClusterGraphT,PointT,PointNT,PointLabelT>::refineUsingCurvature()
{
	/*input: ClusterHandler, ClusterGraph
	 * output:ClusterHandler (refined clusters)
	 * */





	graph_->clusters()->addBorderIndicesToClusters();
	  graph_->clusters()->sortBySize(); // Ascending order
	  ClusterPtr c_it, c_end;
	  for (boost::tie(c_it,c_end) = graph_->clusters()->getClusters(); c_it != c_end; ++c_it)
	  {
	    //std::cout << c_it->size() << std::endl;
	    //if (c_it->size() < 20) continue;
	    if (c_it->type == I_EDGE || c_it->type == I_NAN) continue; //|| c_it->type == I_BORDER) continue;
		graph_->clusters()->computeCurvature(c_it);


	    //computeBoundaryProperties(c_it);
	  }


	  //computeEdgeSmoothness();	//for all edges
	  for ( --c_end; c_end != graph_->clusters()->begin(); --c_end) // iterate from back to front
	  {
	    //if ( c_end->size() < 5 ) continue;
	    if ( c_end->type == I_EDGE || c_end->type == I_NAN) continue;// || c_end->type == I_BORDER) continue;
	    std::vector<ClusterPtr> adj_list;
	    //graph_->getConnectedClusters(c_end->id(), adj_list, graph_->edges()->edge_validator);
	    graph_->getAdjacentClusters(c_end->id(), adj_list);
	    for (typename std::vector<ClusterPtr>::iterator a_it = adj_list.begin(); a_it != adj_list.end(); ++a_it)
	    {
	      if(!similarCurvature(c_end, *a_it)) continue;		//compare smallest segment to adjacent clusters

	    	EdgePtr e = graph_->getConnection(c_end->id(), (*a_it)->id());	//compare smallest segment to all its adjacent clusters
	      //if ( e->smoothness < 0.8 ) continue;
	      //std::cout << sqrt((*a_it)->size()) * 1.0f << " > " << e->size() << std::endl;
	      //if ( e->size() < sqrt((*a_it)->size()) * 0.6f ) continue;	//angrenzende Kante darf nicht zu kurz sein (die beiden Segmente müssen auch wirklich verbunden sein und sich nicht nur in ein paar wenigen Punkten berühren) ???

	      graph_->merge( (*a_it)->id(), c_end->id() );
	      //für merged segments nochmal die curvature berechnen!
	      /*	      std::vector<EdgePtr> updated_edges;
	       * graph_->merge( (*a_it)->id(), c_end->id(), updated_edges );
	      for (typename std::vector<EdgePtr>::iterator e_it = updated_edges.begin(); e_it != updated_edges.end(); ++e_it)
	      {
	  		//graph_->clusters()->computeCurvature(*e_it);
	    	  //computeBoundaryProperties(c_end, *e_it);
	        //computeEdgeSmoothness(*e_it);
	      }*/
	    }
	  }
	  graph_->clusters()->addBorderIndicesToClusters();


}

#endif /* REFINE_SEGMENTATION_HPP_ */
