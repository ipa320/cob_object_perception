/*
 * startVoxelMap.cpp
 *
 *
 *  Created on: 27.03.2013
 *      Author: Matthias Nösner
 */

#include <ros/ros.h>
#include <cob_fiducials/VoxelMap.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "SerializeIO.h"


VoxelMap *v;
//------------main------------------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "VoxelMap");

//	SerializeIO *ser = new SerializeIO("/home/matthias/test.log",'i');

//	ser->writeVariable("myVariable1",20);
//	ser->writeVariable("myVariable2",3.123f);
//	ser->writeVariable("myVariable3",20);
//	ser->writeVariable("myVariable4",3.123f);
//	ser->writeVariable("myVariable5",20);
//	ser->writeVariable("myVariable6",3.123f);
//	ser->writeVariable("myVariable7",20);
//	ser->writeVariable("myVariable8",3.123f);

//	std::vector<std::vector<double> > in;
//	SerializeIO *sertmp = new SerializeIO("/home/matthias/fuerte_workspace/sandbox/cob_object_perception/cob_fiducials/ros/launch/FiducialSimulation/tag6.log",'i');
//	sertmp->openArray("TESTTEST");
//	sertmp->readArray("TESTTEST",&in);
//	sertmp->closeArray();
//	sertmp->close();


	cout << "START - REad" << endl;
	std::vector<std::vector<double> > in;
	SerializeIO *sertmp = new SerializeIO("/home/matthias/fuerte_workspace/sandbox/cob_object_perception/cob_fiducials/ros/launch/FiducialSimulation/tag1EXP.log",'i');
	sertmp->openArray("KickIT");
	sertmp->readArrayEXP("KickIT",&in);
	sertmp->closeArray();
	sertmp->close();
	cout << "Finish - REad" << endl;

	cout << in[7400][3] << endl;


//	cout << "START - Write" << endl;
//	SerializeIO *sertmpo = new SerializeIO("/home/matthias/fuerte_workspace/sandbox/cob_object_perception/cob_fiducials/ros/launch/FiducialSimulation/tag6EXP.log",'o');
//	sertmpo->openArray("KickIT");
//	for(unsigned int i = 0; i < in.size(); i++)
//		sertmpo->writeVectorEXP(in[i]);
//	sertmpo->closeArray();
//	sertmpo->close();
//	cout << "Finish - Write" << endl;


//	std::vector<std::vector<int> > tmp;
//	SerializeIO *sertmpi = new SerializeIO("/home/matthias/EXP.log",'i');
//	sertmpi->openArray("KickIT");
//	sertmpi->readArrayEXP("KickIT",&tmp);
//	sertmpi->closeArray();
//	sertmpi->close();
//
//	cout << tmp[0][0]<< endl;
//	cout << tmp[0][1] << endl;
//	cout << tmp[0][2] << endl;
//	cout << tmp[0][3] << endl;
//
//	cout << tmp[1][0] << endl;
//	cout << tmp[1][1] << endl;
//	cout << tmp[1][2] << endl;
//	cout << tmp[1][3] << endl;

	//v = new VoxelMap();

//	unsigned int id = v->voxelID(-1,5,10);
//	std::cout << "startVoxelMap: POS: " << v->globalmap.voxels[id].pos << std::endl;
//	std::cout << "startVoxelMap: ID:  " << id << std::endl;


    while(ros::ok()){

    	cv::waitKey(10);
    	ros::spinOnce();
    }
}
