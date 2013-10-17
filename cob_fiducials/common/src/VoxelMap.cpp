/*
 * VoxelMap.cpp
 *
 *  Created on: 19.06.2013
 *      Author: matthias
 *
 */

//INFO: All sizes in mm
#include "cob_fiducials/VoxelMap.h"

using namespace std;



//-------------------------------------------------Constructor/Destructor------------------------------------------------------
VoxelMap::VoxelMap() {
	init();
}

VoxelMap::~VoxelMap() {
	// TODO Auto-generated destructor stub
}


//-------------------------------------------------init()----------------------------------------------------------------------
void VoxelMap::init(){
	globalmap.initialized = false;

	cout << "Creating Map...";
 	//Map creation, (Map size, cell size)
	//segmentation fault when marker is detected in bigger distance than Map size!!
	newMap(4000,300);
 	cout << "[READY]" << endl;

 	cout << "Reading Fiducial Marker...";
 	// Position of markers in reference to Marker Connector
	readFiducialMarkers("/home/matthias/fuerte_workspace/sandbox/cob_object_perception/cob_fiducials/ros/launch/Voxelmap/Fiducialsdefine_planar.def",&fiducialmarkers);
	cout << "[READY]" << endl;

	cout << "Fiducial marker size: "<< fiducialmarkers.size() << endl;

	cout << "Reading Points of View...";
	if(!readPointsOfView("/home/matthias/fuerte_workspace/sandbox/cob_object_perception/cob_fiducials/ros/launch/FiducialSimulation/tag1_newerror_TEST.log",&fiducialmarkers[0]))
		std::cout << "VoxelMap.cpp::readPointsofView Failed!!!!!11" <<  std::endl;// Read PoV
	if(!readPointsOfView("/home/matthias/fuerte_workspace/sandbox/cob_object_perception/cob_fiducials/ros/launch/FiducialSimulation/tag2_newerror_TEST.log",&fiducialmarkers[1]))
		std::cout << "VoxelMap.cpp::readPointsofView Failed!!!!!22" <<  std::endl;// Read PoV

//	if(!readPointsOfView("/home/matthias/fuerte_workspace/sandbox/cob_object_perception/cob_fiducials/ros/launch/FiducialSimulation/tag1.log",&fiducialmarkers[0]))
//		std::cout << "VoxelMap.cpp::readPointsofView Failed!!!!!11" <<  std::endl;// Read PoV
//	if(!readPointsOfView("/home/matthias/fuerte_workspace/sandbox/cob_object_perception/cob_fiducials/ros/launch/FiducialSimulation/tag2.log",&fiducialmarkers[1]))
//		std::cout << "VoxelMap.cpp::readPointsofView Failed!!!!!22" <<  std::endl;// Read PoV
//	if(!readPointsOfView("/home/matthias/fuerte_workspace/sandbox/cob_object_perception/cob_fiducials/ros/launch/FiducialSimulation/tag3.log",&fiducialmarkers[2]))
//		std::cout << "VoxelMap.cpp::readPointsofView Failed!!!!!11" <<  std::endl;// Read PoV
//	if(!readPointsOfView("/home/matthias/fuerte_workspace/sandbox/cob_object_perception/cob_fiducials/ros/launch/FiducialSimulation/tag4.log",&fiducialmarkers[3]))
//		std::cout << "VoxelMap.cpp::readPointsofView Failed!!!!!22" <<  std::endl;// Read PoV
//	if(!readPointsOfView("/home/matthias/fuerte_workspace/sandbox/cob_object_perception/cob_fiducials/ros/launch/FiducialSimulation/tag5.log",&fiducialmarkers[4]))
//		std::cout << "VoxelMap.cpp::readPointsofView Failed!!!!!11" <<  std::endl;// Read PoV
//	if(!readPointsOfView("/home/matthias/fuerte_workspace/sandbox/cob_object_perception/cob_fiducials/ros/launch/FiducialSimulation/tag6.log",&fiducialmarkers[5]))
//		std::cout << "VoxelMap.cpp::readPointsofView Failed!!!!!22" <<  std::endl;// Read PoV
	cout << "[READY]" << endl;

	cout << "Assigning POV to Voxel...";
	if(globalmap.initialized) assignPOVToVoxel(&fiducialmarkers[0]);
	if(globalmap.initialized) assignPOVToVoxel(&fiducialmarkers[1]);
//	if(globalmap.initialized) assignPOVToVoxel(&fiducialmarkers[2]);
//	if(globalmap.initialized) assignPOVToVoxel(&fiducialmarkers[3]);
//	if(globalmap.initialized) assignPOVToVoxel(&fiducialmarkers[4]);
//	if(globalmap.initialized) assignPOVToVoxel(&fiducialmarkers[5]);
	cout << "[READY]" << endl;

	findSuperVoxels();

	cout << "[YEAH! I'm ready with all the inefficient Stuff]" << endl;


	cloudhandler = new PCLVisualization();
	drawMarkerImage(&fiducialmarkers[0],cloudhandler);
	drawMarkerImage(&fiducialmarkers[1],cloudhandler);
//	drawMarkerImage(&fiducialmarkers[2],cloudhandler);
//	drawMarkerImage(&fiducialmarkers[3],cloudhandler);
//	drawMarkerImage(&fiducialmarkers[4],cloudhandler);
//	drawMarkerImage(&fiducialmarkers[5],cloudhandler);
	double cloudscale = 1;

	double max_error = 0;
	for(unsigned int i = 0; i < globalmap.voxels.size(); i++){
		for(unsigned int j = 0; j < globalmap.voxels[i].pointalloc.size(); j++){

			fiducialmarker fiducialtmp = getFiducialById(globalmap.voxels[i].pointalloc[j].fiducialID);
			double error = fiducialtmp.points[globalmap.voxels[i].pointalloc[j].pointNumber].e;
			if(error > max_error && fiducialtmp.points[globalmap.voxels[i].pointalloc[j].pointNumber].detected) max_error = error;
		}
	}

	for(unsigned int i = 0; i < globalmap.voxels.size(); i++){

		cv::Mat vec3d_(3,1,CV_64FC1);
		vec3d_.at<double>(0) = globalmap.voxels[i].pos.x;
		vec3d_.at<double>(1) = globalmap.voxels[i].pos.y;
		vec3d_.at<double>(2) = globalmap.voxels[i].pos.z;
		vec3d_ = vec3d_ / cloudscale;
		if(globalmap.voxels[i].pointalloc.size() > 0){
			if(globalmap.voxels[i].issupervoxel){
				cloudhandler->addVecToCloud(vec3d_,255,0,0);
			} else {
				// Show all POV
				//cloudhandler->addVecToCloud(vec3d_,0,255,0);
			}
		} else {
			//cloudhandler->addVecToCloud(vec3d_,255,0,0);
		}
		//show grid
		//cloudhandler->addCube(vec3d_,globalmap.voxelsize/cloudscale);


		for(unsigned int j = 0; j < globalmap.voxels[i].pointalloc.size(); j++){

				fiducialmarker fiducialtmp = getFiducialById(globalmap.voxels[i].pointalloc[j].fiducialID);

				if(fiducialtmp.points[globalmap.voxels[i].pointalloc[j].pointNumber].detected){
					cv::Point3i originPoint = getPointInOriginSystem(&fiducialtmp,globalmap.voxels[i].pointalloc[j].pointNumber);

					cv::Mat vec3d(3,1,CV_64FC1);
					vec3d.at<double>(0) = originPoint.x;
					vec3d.at<double>(1) = originPoint.y;
					vec3d.at<double>(2) = originPoint.z;
					vec3d = vec3d / cloudscale;
					cloudhandler->addVecToCloud(vec3d,0,(int)((255/max_error)*fiducialtmp.points[globalmap.voxels[i].pointalloc[j].pointNumber].e),255);
				}
		}
	}

	cloudhandler->showCloud();

	//TEST
	//output.open ("/home/matthias/Diplomarbeit/GnuPlots/MapTest/maptest.log", std::ios::out | std::ios::app);
	supervoxel = false;
	framenumber = 0;

}

//-------------------------------------------------findSuperVoxels()----------------------------------------------------------------------
// finds supervoxels
// assigns supervoxels to voxels
// assigns nearest supervoxel to voxels

void VoxelMap::findSuperVoxels(){

	std::vector<double> voxelerror(globalmap.voxels.size());

	//loop through all voxels
	for(unsigned int i = 0; i < globalmap.voxels.size(); i++){

		std::vector<std::vector<double> > meanerror(fiducialmarkers.size());
		std::vector<bool> markerinvoxel(fiducialmarkers.size());
		for(unsigned int a=0; a < markerinvoxel.size();a++) markerinvoxel[a] = false;

		//loop through all points
		for(unsigned int j = 0; j < globalmap.voxels[i].pointalloc.size(); j++){
			//compare with all markers for every point
			for(unsigned int k = 0; k < fiducialmarkers.size(); k++){

				//if point belongs to marker and is detected by simulation... save!
				point4d point = getFiducialById(globalmap.voxels[i].pointalloc[j].fiducialID).points[globalmap.voxels[i].pointalloc[j].pointNumber];
				if(globalmap.voxels[i].pointalloc[j].fiducialID == fiducialmarkers[k].id && point.detected){
					markerinvoxel[k] = true;
					meanerror[k].push_back(point.e);
				}
			}
		}

		unsigned int count = 0;
		for(unsigned int k = 0; k < fiducialmarkers.size(); k++){
			if(markerinvoxel[k]) count++;
		}

		//sum up error for each marker in the voxel
		std::vector<fiducialerror> sumederrorvec;
		for(unsigned int k = 0; k < fiducialmarkers.size(); k++){
			if(markerinvoxel[k]){

				fiducialerror ferror;
				ferror.fiducialid = fiducialmarkers[k].id;

				double sumederror = 0;
				for(unsigned int l = 0; l < meanerror[k].size(); l++){
					sumederror += meanerror[k][l];
				}
				ferror.error = sumederror/meanerror[k].size();

				sumederrorvec.push_back(ferror);
			}
		}


		//TEST Normal muss hier > 0 stehen
		if(sumederrorvec.size() > 1){
			//get max element
			double *errors = new double[sumederrorvec.size()];
			for(unsigned int k = 0; k < sumederrorvec.size(); k++){
				//TEST: For Test outcommented!!!!
				//errors[k] = sumederrorvec[k].error;
				//TEST
				voxelerror[i] += sumederrorvec[k].error;
			}
			//TEST
			voxelerror[i] = voxelerror[i]/sumederrorvec.size();

			//TEST: For Test outcommented!!!!
			//write lowest error to voxel_error_map
			//voxelerror[i] = *std::min_element(errors,errors+sumederrorvec.size());
		} else {
			voxelerror[i] = 6666666666666;
		}
	}

	//here its posssible to compute the number of supervoxel with respect to a treshold to the errors
	// here is also the place to add more supervoxel constraints

	unsigned int numberofsupervoxels = 12;
	std::vector<unsigned int> supervoxelid;
	std::vector<double> supervoxelerror;

	supervoxelerror.push_back(voxelerror[0]);
	supervoxelid.push_back(0);

	//SORT voxels in ascending order and save belongings id's
	for(unsigned int i = 1; i < voxelerror.size();i++){

		bool insert = false;
		for(unsigned int j = 0; j < supervoxelerror.size() && !insert;j++){

			if(voxelerror[i] < supervoxelerror[j]){
				supervoxelerror.insert(supervoxelerror.begin()+j,voxelerror[i]);
				supervoxelid.insert(supervoxelid.begin()+j,i);
				insert = true;
			}
		}

		if(!insert) {
			supervoxelerror.push_back(voxelerror[i]);
			supervoxelid.push_back(i);
		}

		//cout << "Prepared " << i << "/" << voxelerror.size() << endl;
	}

	//debug********************************************
	for(unsigned int i = 0; i < numberofsupervoxels;i++){
		cout << supervoxelid[i] << "......." << supervoxelerror[i] << endl;
	}

	cout << "...................................................." << endl;

//	for(unsigned int i = 10638; i < 10648;i++){
//		cout << supervoxelid[i] << "......." << supervoxelerror[i] << endl;
//	}
	//debug********************************************

	//TODO save supervoxels to voxels.... but compute distances between voxels.

	//Optionally -> Save all supervoxelid'S to each voxel
	for(unsigned int i = 0; i < globalmap.voxels.size() ;i++){
		globalmap.voxels[i].issupervoxel = false;

		for(unsigned int j = 0; j < numberofsupervoxels;j++){
			globalmap.voxels[i].premiumvoxels.push_back(supervoxelid[j]);

			if(globalmap.voxels[i].id == supervoxelid[j])
				globalmap.voxels[i].issupervoxel = true;
		}
	}

	//precompute the nearest voxel
	for(unsigned int i = 0; i < globalmap.voxels.size();i++){

		unsigned int nearestid = globalmap.voxels[i].premiumvoxels[0];
		double smallest_distance = sqrt(globalmap.voxels[i].pos.x*globalmap.voxels[nearestid].pos.x +
										 globalmap.voxels[i].pos.y*globalmap.voxels[nearestid].pos.y +
										 globalmap.voxels[i].pos.z*globalmap.voxels[nearestid].pos.z );

		for(unsigned int j = 1; j < globalmap.voxels[i].premiumvoxels.size(); j++){

			double distance =sqrt(globalmap.voxels[i].pos.x*globalmap.voxels[globalmap.voxels[i].premiumvoxels[j]].pos.x +
								   globalmap.voxels[i].pos.y*globalmap.voxels[globalmap.voxels[i].premiumvoxels[j]].pos.y +
								   globalmap.voxels[i].pos.z*globalmap.voxels[globalmap.voxels[i].premiumvoxels[j]].pos.z );
			if(distance < smallest_distance){
				smallest_distance = distance;
				nearestid = globalmap.voxels[i].premiumvoxels[j];
			}
		}
		globalmap.voxels[i].nextpremiumvoxel = nearestid;
	}
}



//-------------------------------------------------readFiducialMarker()----------------------------------------------------------------------
bool VoxelMap::readFiducialMarkers(const char* filename,vector<fiducialmarker>* afiducialmarkers){

	SerializeIO *ser = new SerializeIO(filename,'i');

	fiducialmarker fiducial1,fiducial2,fiducial3,fiducial4,fiducial5, fiducial6;

	if(!ser->readVariable("fiducial1_ID",&(fiducial1.id))) return false;
	if(!ser->readVariable("fiducial1_POSX",&(fiducial1.trans.x))) return false;
	if(!ser->readVariable("fiducial1_POSY",&(fiducial1.trans.y))) return false;
	if(!ser->readVariable("fiducial1_POSZ",&(fiducial1.trans.z))) return false;
	if(!ser->readVariable("fiducial1_ROT1",&(fiducial1.rot.x))) return false;
	if(!ser->readVariable("fiducial1_ROT2",&(fiducial1.rot.y))) return false;
	if(!ser->readVariable("fiducial1_ROT3",&(fiducial1.rot.z))) return false;

	if(!ser->readVariable("fiducial2_ID",&(fiducial2.id))) return false;
	if(!ser->readVariable("fiducial2_POSX",&(fiducial2.trans.x))) return false;
	if(!ser->readVariable("fiducial2_POSY",&(fiducial2.trans.y))) return false;
	if(!ser->readVariable("fiducial2_POSZ",&(fiducial2.trans.z))) return false;
	if(!ser->readVariable("fiducial2_ROT1",&(fiducial2.rot.x))) return false;
	if(!ser->readVariable("fiducial2_ROT2",&(fiducial2.rot.y))) return false;
	if(!ser->readVariable("fiducial2_ROT3",&(fiducial2.rot.z))) return false;

//	if(!ser->readVariable("fiducial3_ID",&(fiducial3.id))) return false;
//	if(!ser->readVariable("fiducial3_POSX",&(fiducial3.trans.x))) return false;
//	if(!ser->readVariable("fiducial3_POSY",&(fiducial3.trans.y))) return false;
//	if(!ser->readVariable("fiducial3_POSZ",&(fiducial3.trans.z))) return false;
//	if(!ser->readVariable("fiducial3_ROT1",&(fiducial3.rot.x))) return false;
//	if(!ser->readVariable("fiducial3_ROT2",&(fiducial3.rot.y))) return false;
//	if(!ser->readVariable("fiducial3_ROT3",&(fiducial3.rot.z))) return false;
//
//	if(!ser->readVariable("fiducial4_ID",&(fiducial4.id))) return false;
//	if(!ser->readVariable("fiducial4_POSX",&(fiducial4.trans.x))) return false;
//	if(!ser->readVariable("fiducial4_POSY",&(fiducial4.trans.y))) return false;
//	if(!ser->readVariable("fiducial4_POSZ",&(fiducial4.trans.z))) return false;
//	if(!ser->readVariable("fiducial4_ROT1",&(fiducial4.rot.x))) return false;
//	if(!ser->readVariable("fiducial4_ROT2",&(fiducial4.rot.y))) return false;
//	if(!ser->readVariable("fiducial4_ROT3",&(fiducial4.rot.z))) return false;
//
//	if(!ser->readVariable("fiducial5_ID",&(fiducial5.id))) return false;
//	if(!ser->readVariable("fiducial5_POSX",&(fiducial5.trans.x))) return false;
//	if(!ser->readVariable("fiducial5_POSY",&(fiducial5.trans.y))) return false;
//	if(!ser->readVariable("fiducial5_POSZ",&(fiducial5.trans.z))) return false;
//	if(!ser->readVariable("fiducial5_ROT1",&(fiducial5.rot.x))) return false;
//	if(!ser->readVariable("fiducial5_ROT2",&(fiducial5.rot.y))) return false;
//	if(!ser->readVariable("fiducial5_ROT3",&(fiducial5.rot.z))) return false;
//
//	if(!ser->readVariable("fiducial6_ID",&(fiducial6.id))) return false;
//	if(!ser->readVariable("fiducial6_POSX",&(fiducial6.trans.x))) return false;
//	if(!ser->readVariable("fiducial6_POSY",&(fiducial6.trans.y))) return false;
//	if(!ser->readVariable("fiducial6_POSZ",&(fiducial6.trans.z))) return false;
//	if(!ser->readVariable("fiducial6_ROT1",&(fiducial6.rot.x))) return false;
//	if(!ser->readVariable("fiducial6_ROT2",&(fiducial6.rot.y))) return false;
//	if(!ser->readVariable("fiducial6_ROT3",&(fiducial6.rot.z))) return false;

	afiducialmarkers->push_back(fiducial1);
	afiducialmarkers->push_back(fiducial2);
//	afiducialmarkers->push_back(fiducial3);
//	afiducialmarkers->push_back(fiducial4);
//	afiducialmarkers->push_back(fiducial5);
//	afiducialmarkers->push_back(fiducial6);

	ser->close();
	delete ser;
	return true;
}


//-------------------------------------------------readPointsOfView()----------------------------------------------------------------------
// Reads all Points of View for a specific Fiducial Marker
bool VoxelMap::readPointsOfView(const char* filename,fiducialmarker* afiducialmarkers){

	SerializeIO *ser = new SerializeIO(filename,'i');

	vector<vector<double> > tmp;

	ser->openArray("TESTTEST");
	if(!ser->readArray("TESTTEST",&tmp)) return false;
	ser->closeArray();
	ser->close();

	for(unsigned int i = 0; i < tmp.size(); i++){
		afiducialmarkers->points.push_back(transformToPoint4d(tmp[i]));
	}

	delete ser;
	return true;
}


//-------------------------------------------------readPointsOfView()----------------------------------------------------------------------
// Transformation from camera system to marker system
point4d VoxelMap::transformToPoint4d(vector<double> vec){

	if(vec.size() < 7){
		cout << "VoxelMap::transformToPoint4d(vector<double> vec): " << "Wrong size of Vector ... check the infile" << endl;
	}

	cv::Mat trans(3,1,CV_64FC1);
	trans.at<double>(0) = vec[0];
	trans.at<double>(1) = vec[1];
	trans.at<double>(2) = -vec[2]*1000;

	cv::Mat rotPointToMarkerSys = eulerToMatrixFiducialSystem(vec[3],vec[4],vec[5]);

	cv::Mat point3d(3,1,CV_64FC1);
	point3d = rotPointToMarkerSys.t()*trans;

	point4d ret;
	ret.x = point3d.at<double>(0);
	ret.y = point3d.at<double>(1);
	ret.z = point3d.at<double>(2);

	//TODO This is just a example think about the error!!!!
	//ret.e = vec[6]+vec[2]*sin(vec[7]*PI/180);
	ret.e = vec[6];

	ret.detected = true;
	if(vec[7] == -666) ret.detected = false;

	return ret;
}

//-------------------------------------------------assignPOVToVoxel()----------------------------------------------------------------------
//Assigns all Points in the Fiducial Marker coordinate System to the Voxels
bool VoxelMap::assignPOVToVoxel(fiducialmarker* afiducialmarker){

	for(unsigned int i = 0; i < afiducialmarker->points.size(); i++){


		cv::Point3d pointFromOrigin= getPointInOriginSystem(afiducialmarker,i);


		unsigned int id = voxelID(pointFromOrigin.x,pointFromOrigin.y,pointFromOrigin.z);
		pointallocation palloc;
		palloc.fiducialID = afiducialmarker->id;
		palloc.pointNumber = i;

		globalmap.voxels[id].pointalloc.push_back(palloc);
	}

	return false;
}

//-------------------------------------------------getPointInOriginSystem()----------------------------------------------------------------------
//Transforms Point from Fiducial Marker System to Origin System
cv::Point3d VoxelMap::getPointInOriginSystem(fiducialmarker* afiducialmarker,unsigned int pointnumber){

	cv::Mat transOrigin(3,1,CV_64FC1);
	transOrigin.at<double>(0) = afiducialmarker->trans.x;
	transOrigin.at<double>(1) = afiducialmarker->trans.y;
	transOrigin.at<double>(2) = afiducialmarker->trans.z;

	cv::Mat rotOrigin = eulerToMatrixOriginSystem(afiducialmarker->rot.x,afiducialmarker->rot.y,afiducialmarker->rot.z);

	//point in Fiducial System
	cv::Mat point3d(3,1,CV_64FC1);
	point3d.at<double>(0) = afiducialmarker->points[pointnumber].x;
	point3d.at<double>(1) = afiducialmarker->points[pointnumber].y;
	point3d.at<double>(2) = afiducialmarker->points[pointnumber].z;

	// + offset to Origin
	point3d = rotOrigin*point3d;
	point3d += transOrigin;
    //point3d is now in Origin System!!

	cv::Point3d ret;
	ret.x = point3d.at<double>(0);
	ret.y = point3d.at<double>(1);
	ret.z = point3d.at<double>(2);

	return ret;
}


//-------------------------------------------------getFiducialById()----------------------------------------------------------------------
fiducialmarker VoxelMap::getFiducialById(unsigned int id){

	for(unsigned int i = 0; i < fiducialmarkers.size(); i++){
		if(id == fiducialmarkers[i].id) return fiducialmarkers[i];
	}

	cout << "VoxelMap::getFiducialById(unsigned int id): " << "Something went wrong. Could not resolve Fiducial Id.. " << endl;

	return fiducialmarker();
}

//-------------------------------------------------setDetectedMarkerArray()----------------------------------------------------------------------
//uses fiducial with lowest error to display camera position
void VoxelMap::setDetectedMarkerArray(std::vector<std::vector<double> > markerarray){

	//vector struct
	//***********
	//id
	//x
	//y
	//z
	//alpha
	//beta
	//gamma
	//****************

	bool voxelpopulated = false;
	double act_error = 10000000;
	unsigned int markerarraypos = 0;

	cv::Point3d marker1,marker2;

	//TEST
	for(unsigned int i = 0; i < markerarray.size(); i++){
		if((unsigned int)markerarray[i][0] == 1){
			marker1.x = markerarray[i][1];
			marker1.y = markerarray[i][2];
			marker1.z = markerarray[i][3];
		}
		if((unsigned int)markerarray[i][0] == 2){
			marker2.x = markerarray[i][1];
			marker2.y = markerarray[i][2];
			marker2.z = markerarray[i][3];
		}
	}

	double distance = sqrt((marker1.x-marker2.x)*(marker1.x-marker2.x) +
							(marker1.y-marker2.y)*(marker1.y-marker2.y) +
							(marker1.z-marker2.z)*(marker1.z-marker2.z) );
	cout << "[DISTANCE]: " << distance << endl;
	if(true){
		output.open ("/home/matthias/Diplomarbeit/GnuPlots/MapTest/2ndTest/notsuper4.log", std::ios::out | std::ios::app | std::ios::ate);
		stringstream ss;
		ss << framenumber << " " << distance << "\n";
		output << ss.str();
		output.close();
		framenumber++;
	}
	//TEST

	for(unsigned int i = 0; i < markerarray.size(); i++){

		unsigned int fiducialid = (unsigned int)markerarray[i][0];
		std::vector<cv::Mat> campos = getCameraPOS(fiducialid,markerarray[i][1],markerarray[i][2],markerarray[i][3],markerarray[i][4],markerarray[i][5],markerarray[i][6]);

	    cv::Mat cameraUPVector = campos[0];
	    cv::Mat cameraTranslationVector = campos[1];
	    cv::Mat cameraOrientationVector = campos[2];

	   unsigned int voxelid = voxelID(cameraTranslationVector.at<double>(0),cameraTranslationVector.at<double>(1),cameraTranslationVector.at<double>(2));

	   cout << "[DEBUG] The Actual Position is [Reference to Fiducial: " << fiducialid
		   <<  "]: [" << cameraTranslationVector.at<double>(0) << "]"
			   << "[" << cameraTranslationVector.at<double>(1) << "]"
			   << "[" << cameraTranslationVector.at<double>(2) << "]" << endl;

	   for(unsigned int j =0; j < globalmap.voxels[voxelid].pointalloc.size(); j++){

		   unsigned int fiducialid_ = globalmap.voxels[voxelid].pointalloc[j].fiducialID;
		   unsigned int pointnumber = globalmap.voxels[voxelid].pointalloc[j].pointNumber;

		   //compare if same markers
		   if(fiducialid_ == fiducialid){
			   //get the one with lowest error
			   if(getFiducialById(fiducialid_).points[pointnumber].e < act_error){
				   act_error = getFiducialById(fiducialid_).points[pointnumber].e;
				   markerarraypos = i;
				   voxelpopulated = true;
			   }
		   }
	   }
	}

	if( markerarray.size() > 0){
		//Handle supervoxelstuff
		//Maybe the distance to a supervoxel can also be added ... to make it easier reaching the area of a supervoxel
		unsigned int fiducialid = (unsigned int)markerarray[markerarraypos][0];
		std::vector<cv::Mat> campos = getCameraPOS(fiducialid,markerarray[markerarraypos][1],markerarray[markerarraypos][2],markerarray[markerarraypos][3],markerarray[markerarraypos][4],markerarray[markerarraypos][5],markerarray[markerarraypos][6]);
		cv::Mat cameraUPVector = campos[0];
		cv::Mat cameraTranslationVector = campos[1];
		cv::Mat cameraOrientationVector = campos[2];

		unsigned int voxelid = voxelID(cameraTranslationVector.at<double>(0),cameraTranslationVector.at<double>(1),cameraTranslationVector.at<double>(2));

		if(globalmap.voxels[voxelid].issupervoxel){
			cout << "YEAH: I'm in Position to Grab the Sonde!!!!!!!!!!!!!!!" << endl;
			supervoxel = true;
		} else {
			supervoxel = false;
			cout << "The next Supervoxel is at Position: [" << globalmap.voxels[globalmap.voxels[voxelid].nextpremiumvoxel].pos.x << "]" <<
					"[" << globalmap.voxels[globalmap.voxels[voxelid].nextpremiumvoxel].pos.y << "]" <<
					"[" << globalmap.voxels[globalmap.voxels[voxelid].nextpremiumvoxel].pos.z << "]" << endl;

			cout << "The Actual Position is [Reference to Fiducial: " << fiducialid <<  "]: [" << cameraTranslationVector.at<double>(0) << "]" << "[" << cameraTranslationVector.at<double>(1) << "]" << "[" << cameraTranslationVector.at<double>(2) << "]" << endl;
		}


	    //debug stuff (Visualization)
		cloudhandler->addRealtimeInformation(cameraTranslationVector,cameraOrientationVector,cameraUPVector,50);

		//	cloudhandler->setCameraToPOV(cameraTranslationVector.at<double>(0),cameraTranslationVector.at<double>(1),cameraTranslationVector.at<double>(2),
		//    		                     cameraOrientationVector.at<double>(0),cameraOrientationVector.at<double>(1),cameraOrientationVector.at<double>(2),
		//    		                     cameraUPVector.at<double>(0),cameraUPVector.at<double>(1),cameraUPVector.at<double>(2));
	    //debug stuff
	}

//	if(voxelpopulated){
//
//		std::vector<cv::Mat> campos = getCameraPOS((int)markerarray[markerarraypos][0],markerarray[markerarraypos][1],markerarray[markerarraypos][2],markerarray[markerarraypos][3],
//														markerarray[markerarraypos][4],markerarray[markerarraypos][5],markerarray[markerarraypos][6]);
//		cv::Mat cameraUPVector = campos[0];
//		cv::Mat cameraTranslationVector = campos[1];
//		cv::Mat cameraOrientationVector = campos[2];
//
//
//	    //debug stuff (Visualization)
//		cloudhandler->addRealtimeInformation(cameraTranslationVector,cameraOrientationVector,cameraUPVector,50);
//
//		//	cloudhandler->setCameraToPOV(cameraTranslationVector.at<double>(0),cameraTranslationVector.at<double>(1),cameraTranslationVector.at<double>(2),
//		//    		                     cameraOrientationVector.at<double>(0),cameraOrientationVector.at<double>(1),cameraOrientationVector.at<double>(2),
//		//    		                     cameraUPVector.at<double>(0),cameraUPVector.at<double>(1),cameraUPVector.at<double>(2));
//	    //debug stuff
//	}
}

//-------------------------------------------------setCameraToPOV()----------------------------------------------------------------------
//Do all the coordinate transformation and return the camera position and orientation
std::vector<cv::Mat> VoxelMap::getCameraPOS(int id,double x,double y,double z,double alpha,double beta,double gamma){

	double conversion = 1000 ;

	cv::Mat cameraOriginWorld(3,1,CV_64FC1);
	cv::Mat markerOriginWorld(3,1,CV_64FC1);

	cv::Mat objectRotation_vec(3,1,CV_64FC1);
	objectRotation_vec.at<double>(0) = alpha;
	objectRotation_vec.at<double>(1) = beta;
	objectRotation_vec.at<double>(2) = gamma;

	cv::Mat objectTranslation_vec(3,1,CV_64FC1);
	objectTranslation_vec.at<double>(0) = x;
	objectTranslation_vec.at<double>(1) = y;
	objectTranslation_vec.at<double>(2) = z;

    cv::Mat R;
    cv::Rodrigues(objectRotation_vec, R);


//    cv::Mat cameraTranslationVector(3,1,CV_64FC1);
//    cameraTranslationVector.at<double>(0) = -objectTranslation_vec.at<double>(0);
//    cameraTranslationVector.at<double>(1) =  objectTranslation_vec.at<double>(1);
//    cameraTranslationVector.at<double>(2) =  objectTranslation_vec.at<double>(2);

    cv::Mat cameraTranslationVector(3,1,CV_64FC1);
    cameraTranslationVector.at<double>(0) =  0;
    cameraTranslationVector.at<double>(1) =  0;
    cameraTranslationVector.at<double>(2) =  -sqrt(z*z+y*y+x*x);
    cameraTranslationVector = R.t()*cameraTranslationVector;

    cv::Mat cameraOrientationVector(3,1,CV_64FC1);
    cameraOrientationVector.at<double>(0) = 0;
    cameraOrientationVector.at<double>(1) = 0;
    cameraOrientationVector.at<double>(2) = 1;

    cv::Mat cameraUPVector(3,1,CV_64FC1);
    cameraUPVector.at<double>(0) = 0;
    cameraUPVector.at<double>(1) = -1;
    cameraUPVector.at<double>(2) = 0;

    //----------------Transformation Camera in Marker System ----------------------------------------------------
    //Computing the Camera View xdirection in Marker System
    cameraOrientationVector = R.t()*cameraOrientationVector;
    cameraOrientationVector += cameraTranslationVector;

    //Computing the Camera Up vector in Marker System
    cameraUPVector = R.t()*cameraUPVector;
    cameraUPVector += cameraTranslationVector;
    //-----------------Transformation from Camera in Marker System ----------------------------------------------------


    //----- Transformation to World System------------------------------------------------------------------------------
    fiducialmarker fiducial = getFiducialById((unsigned int)id);

    cv::Mat transOrigin(3,1,CV_64FC1);
    transOrigin.at<double>(0) = fiducial.trans.x;
    transOrigin.at<double>(1) = fiducial.trans.y;
    transOrigin.at<double>(2) = fiducial.trans.z;
    transOrigin = transOrigin/conversion; //from mm -> m
    cv::Mat rotOrigin = eulerToMatrixOriginSystem(fiducial.rot.x,fiducial.rot.y,fiducial.rot.z);


    cameraTranslationVector = rotOrigin*cameraTranslationVector;
    cameraOrientationVector = rotOrigin*cameraOrientationVector;
    cameraUPVector = rotOrigin*cameraUPVector;

    cameraTranslationVector += transOrigin;
    cameraOrientationVector += transOrigin;
    cameraUPVector += transOrigin;
    //----- Transformation to World System------------------------------------------------------------------------------


    //Point cloud scaling -> from m -> mm
    cameraUPVector = cameraUPVector*conversion;
    cameraTranslationVector = cameraTranslationVector*conversion;
    cameraOrientationVector = cameraOrientationVector*conversion;

    std::vector<cv::Mat> ret;
    ret.push_back(cameraUPVector);
    ret.push_back(cameraTranslationVector);
    ret.push_back(cameraOrientationVector);

    return ret;

}

//-------------------------------------------------spin()----------------------------------------------------------------------
//To give the Point Cloud Lib computing Time.. otherwise the pos of camera can only be moved when a marker is detected
void VoxelMap::spin(){
	cloudhandler->spin();
}
//-------------------------------------------------drawMarkerImage()----------------------------------------------------------------------
void VoxelMap::drawMarkerImage(fiducialmarker* afiducialmarker,PCLVisualization* cloudhandler){

	cv::Mat *image;
	stringstream ss;
	ss << afiducialmarker->id;
	image = new cv::Mat(cv::imread("/home/matthias/fuerte_workspace/sandbox/cob_object_perception/cob_fiducials/ros/launch/Voxelmap/planarmarker" + ss.str() + ".png"));


	cv::Point imagecenter(image->rows/2,image->cols/2);
	int r,g,b;
	for (int j = 0; j < image->rows; j++){
		for (int h = 0; h < image->cols; h++){

			try{
				r = image->at<cv::Vec3b>(h,j)[0];
				g = image->at<cv::Vec3b>(h,j)[1];
				b = image->at<cv::Vec3b>(h,j)[2];
			} catch(std::exception& e) {
				cout << "ERROR:" << e.what() << endl;
			}

			cv::Mat vec__3d(3,1,CV_64F);
			vec__3d.at<double>(0) = (-imagecenter.x+j)/5;
			vec__3d.at<double>(1) = ( imagecenter.y-h)/5;
			vec__3d.at<double>(2) = 1/10;


			cv::Mat transOrigin(3,1,CV_64FC1);
			transOrigin.at<double>(0) = afiducialmarker->trans.x;
			transOrigin.at<double>(1) = afiducialmarker->trans.y;
			transOrigin.at<double>(2) = afiducialmarker->trans.z;

			cv::Mat rotOrigin = eulerToMatrixOriginSystem(afiducialmarker->rot.x,afiducialmarker->rot.y,afiducialmarker->rot.z);

			vec__3d = rotOrigin*vec__3d;
			vec__3d += transOrigin;

			cloudhandler->addVecToCloud(vec__3d,r,g,b);
		}
	}

	image->~Mat();
}

//-------------------------------------------------newMap()----------------------------------------------------------------------
bool VoxelMap::newMap(int mapsize, int voxelsize){

	//Set to computable(even) values
	voxelsize = voxelsize - (voxelsize%2);
	mapsize = mapsize - (mapsize%voxelsize);
	cout << "VoxelMap: voxelsize was set to: " << voxelsize << "mm" << endl;
	cout << "VoxelMap: maximum_map_distance was set to: " << mapsize << "mm" <<endl;

	//calculate number of voxels
	unsigned int voxelnumber_line = (unsigned int) (2*mapsize/voxelsize);
	unsigned int voxelnumber_global = voxelnumber_line*voxelnumber_line*voxelnumber_line;

	unsigned int idcounter = 0;

	int fvoxel = -mapsize + voxelsize/2;
	int lvoxel =  mapsize;
	int stepsize = voxelsize;

	//Create the voxels from -x,-y,-z -> x,y,z in ordering x,y,z
	for(int z = fvoxel; z < lvoxel ; z = z + stepsize){
		for(int y = fvoxel; y < lvoxel ; y = y + stepsize){
			for(int x = fvoxel; x < lvoxel ; x = x + stepsize){
				voxel tmp;
				tmp.id = idcounter;
				tmp.pos = cv::Point3i(x,y,z);
				tmp.size = cv::Scalar(voxelsize/2,voxelsize/2,voxelsize/2,0);
				globalmap.voxels.push_back(tmp);
				idcounter++;
			}
		}
	}

	cout << "Voxelmap: Number of Voxels in map: " << globalmap.voxels.size() << endl;

	if(globalmap.voxels.size() != voxelnumber_global){
		cout << "VoxelMap: newMap: Something went wrong!!" << endl;
		return false;
	}

	globalmap.size = mapsize;
	globalmap.voxelsize = voxelsize;
	globalmap.initialized = true;

	return true;
}

//-------------------------------------------------voxelId(int x, int y, int z)---------------------------------------------------
// compute voxel from a given point in the continous 3d space
unsigned int VoxelMap::voxelID(double x, double y, double z){

	int mapsize = globalmap.size;
	int voxelsize = globalmap.voxelsize;
	int voxelnumber_dim = 2*mapsize/voxelsize;

	double fvoxel = -mapsize + voxelsize/2;
	//everything ok till here

	int stepsx = cvRound( (x - fvoxel)/voxelsize );
	int stepsy = cvRound( (y - fvoxel)/voxelsize );
	int stepsz = cvRound( (z - fvoxel)/voxelsize );

	stepsx = sqrt(stepsx*stepsx);
	stepsy = sqrt(stepsy*stepsy);
	stepsz = sqrt(stepsz*stepsz);

	unsigned int id = stepsz * voxelnumber_dim*voxelnumber_dim + stepsy * voxelnumber_dim + stepsx;

	return id;
}

//-----------------------------------additional Stuff (copied from FiducialmarkerevaluationToolkit)--------------------------------------------------------------------------------
// Transformation from camera system to marker system
cv::Mat VoxelMap::eulerToMatrixFiducialSystem(double rot1, double rot2, double rot3) {

	cv::Mat m(3,3,CV_64FC1);

	cv::Mat X_rot(3,3,CV_64F);
	cv::Mat Y_rot(3,3,CV_64F);
	cv::Mat Z_rot(3,3,CV_64F);

	double alpha =  rot1 - PI;
	double beta  =  rot2;
	double gamma =  rot3;

	//Achtung auf Rotationsreihenfolge achten....  sonst stimmt die Darstellung nicht
	X_rot.at<double>(0,0) =  1.0;
	X_rot.at<double>(0,1) =  0.0;
	X_rot.at<double>(0,2) =  0.0;
	X_rot.at<double>(1,0) =  0.0;
	X_rot.at<double>(1,1) =  cos(alpha);
	X_rot.at<double>(1,2) =  -sin(alpha);
	X_rot.at<double>(2,0) =  0.0;
	X_rot.at<double>(2,1) =  sin(alpha);
	X_rot.at<double>(2,2) =  cos(alpha);

	Y_rot.at<double>(0,0) =  cos(beta);
	Y_rot.at<double>(0,1) =  0.0;
	Y_rot.at<double>(0,2) =  sin(beta);
	Y_rot.at<double>(1,0) =  0.0;
	Y_rot.at<double>(1,1) =  1.0;
	Y_rot.at<double>(1,2) =  0.0;
	Y_rot.at<double>(2,0) =  -sin(beta);
	Y_rot.at<double>(2,1) =  0.0;
	Y_rot.at<double>(2,2) =  cos(beta);

	Z_rot.at<double>(0,0) =  cos(gamma);
	Z_rot.at<double>(0,1) =  -sin(gamma);
	Z_rot.at<double>(0,2) =  0.0;
	Z_rot.at<double>(1,0) =  sin(gamma);
	Z_rot.at<double>(1,1) =  cos(gamma);
	Z_rot.at<double>(1,2) =  0.0;
	Z_rot.at<double>(2,0) =  0.0;
	Z_rot.at<double>(2,1) =  0.0;
	Z_rot.at<double>(2,2) =  1.0;

	m = X_rot*Z_rot;

	return m;
}

cv::Mat VoxelMap::eulerToMatrixOriginSystem(double rot1, double rot2, double rot3) {

	cv::Mat m(3,3,CV_64FC1);

	cv::Mat X_rot(3,3,CV_64F);
	cv::Mat Y_rot(3,3,CV_64F);
	cv::Mat Z_rot(3,3,CV_64F);

	double alpha =  rot1;
	double beta  =  rot2;
	double gamma =  rot3;


	X_rot.at<double>(0,0) =  1.0;
	X_rot.at<double>(0,1) =  0.0;
	X_rot.at<double>(0,2) =  0.0;
	X_rot.at<double>(1,0) =  0.0;
	X_rot.at<double>(1,1) =  cos(alpha);
	X_rot.at<double>(1,2) =  -sin(alpha);
	X_rot.at<double>(2,0) =  0.0;
	X_rot.at<double>(2,1) =  sin(alpha);
	X_rot.at<double>(2,2) =  cos(alpha);

	Y_rot.at<double>(0,0) =  cos(beta);
	Y_rot.at<double>(0,1) =  0.0;
	Y_rot.at<double>(0,2) =  sin(beta);
	Y_rot.at<double>(1,0) =  0.0;
	Y_rot.at<double>(1,1) =  1.0;
	Y_rot.at<double>(1,2) =  0.0;
	Y_rot.at<double>(2,0) =  -sin(beta);
	Y_rot.at<double>(2,1) =  0.0;
	Y_rot.at<double>(2,2) =  cos(beta);

	Z_rot.at<double>(0,0) =  cos(gamma);
	Z_rot.at<double>(0,1) =  -sin(gamma);
	Z_rot.at<double>(0,2) =  0.0;
	Z_rot.at<double>(1,0) =  sin(gamma);
	Z_rot.at<double>(1,1) =  cos(gamma);
	Z_rot.at<double>(1,2) =  0.0;
	Z_rot.at<double>(2,0) =  0.0;
	Z_rot.at<double>(2,1) =  0.0;
	Z_rot.at<double>(2,2) =  1.0;

	m = Z_rot*Y_rot*X_rot;

	return m;
}
