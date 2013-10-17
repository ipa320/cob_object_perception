/*
 * fiducialsvalidation.cpp
 * I'm to lazy to rename it.... but its right now just for preparing the candlestickdata
 *  Created on: 27.03.2013
 *      Author: Matthias Nösner
 *
 *      ACHTUNG !!!!!! DATA CONVERSION CLASS !!! NOT GOOD COMMENTED !!! DONT USE!!!
 */

#include <ros/ros.h>
#include <opencv/cv.h>
#include "opencv/highgui.h"
#include <SerializeIO.h>

unsigned int counter = 0;

using namespace std;

const double PI = 3.14159265358979323843;

void candlesticks();
void prepareDataforFOV();
void prepareDataforFOV2();
void chooseErrorPoints();

void candlesticks(){
	SerializeIO *ser = new SerializeIO("/media/DATEN/Extended_PiTag10/Results/blured15/pointcloudID3.log",'i');

	fstream output;
	output.open("/media/DATEN/Extended_PiTag10/Results/blured15/candledata_rot.log",ios_base::out);

	double conversion = 1;//* conversion

	vector<vector<double> > tmp;
	vector<double> candle;

	ser->openArray("TESTTEST");
	if(!ser->readArray("TESTTEST",&tmp)) cout << "Something Went Wrong" << endl;
	ser->closeArray();
	ser->close();

	int fieldnumber = 7;//trans(6)//rot(7)

	double lastdepth =  tmp[0][2];

	for(unsigned int i = 0; i < tmp.size(); i++){
		cout << "Progress: " << i << "/" << tmp.size() << endl;
		if(lastdepth < tmp[i][2] || i == tmp.size()-1){ // depth is responsible factor
			//evaluate
			//kill all nonvalid data
			vector<double> candletmp;
			for(unsigned int j = 0; j < candle.size(); j++){
				if(candle[j] != -666) candletmp.push_back(candle[j]); // write only valid data -> -666 means .. no marker detected
			}
			//compute max,min,mean
			double max_elem = 0;
			double min_elem = 0;
			double sum = 0;

			if(candletmp.size() > 0){
				max_elem = candletmp[0];
				min_elem = candletmp[0];
				sum = candletmp[0];
			}
			for(unsigned int j = 1; j < candletmp.size(); j++){
				if(candletmp[j] > max_elem) max_elem = candletmp[j];
				if(candletmp[j] < min_elem) min_elem = candletmp[j];
				sum += candletmp[j];
			}
			double mean = 0;
			if(candletmp.size() > 0) mean  = sum / candletmp.size();

			//compute best 90%  for big candle
			vector<double> distancetomean;
			for(unsigned int j = 0; j < candletmp.size(); j++){
				distancetomean.push_back(sqrt((mean-candletmp[j])*(mean-candletmp[j])));
			}
			//sort in ascending order
			int percentage = 90;
			std::vector<unsigned int> id;
			std::vector<double> distancetomean_asc;

			if(distancetomean.size() > 0){
				distancetomean_asc.push_back(distancetomean[0]);
				id.push_back(0);
			}
			for(unsigned int i = 1; i < distancetomean.size();i++){
				bool insert = false;
				for(unsigned int j = 0; j < distancetomean_asc.size() && !insert;j++){
					if(distancetomean[i] < distancetomean_asc[j]){
						distancetomean_asc.insert(distancetomean_asc.begin()+j,distancetomean[i]);
						id.insert(id.begin()+j,i);
						insert = true;
					}
				}

				if(!insert) {
					distancetomean_asc.push_back(distancetomean[i]);
					id.push_back(i);
				}
			}
			//write first 90% to tmp
			unsigned int numberofvelementsin90 = (unsigned int)cvRound(distancetomean.size()*9/10);
			vector<double> tmp_;
			for(unsigned int i = 0; i < numberofvelementsin90;i++){
				tmp_.push_back(candletmp[id[i]]);
			}
			//get the min max value of the big candle
			double max_elem_candle = 0;
			double min_elem_candle = 0;
			if(candletmp.size() > 0){
				max_elem_candle = candletmp[0];
				min_elem_candle = candletmp[0];
			}

			for(unsigned int j = 1; j < tmp_.size(); j++){
				if(tmp_[j] > max_elem_candle) max_elem_candle = tmp_[j];
				if(tmp_[j] < min_elem_candle) min_elem_candle = tmp_[j];
			}

			//make ready for next iteration
			candle.clear();
			//WRITE DATA TO FILE

			stringstream ss;

			ss << lastdepth << " " << min_elem*conversion << " "  << min_elem_candle*conversion << " " << mean*conversion << " " << max_elem_candle*conversion << " " << max_elem*conversion << endl;
			output << ss.str();

		}
		lastdepth = tmp[i][2]; //depth
		candle.push_back(tmp[i][fieldnumber]); // euc dist
	}

	delete ser;
}


void prepareDataforFOV(){

	SerializeIO *ser = new SerializeIO("/media/DATEN/Extended_PiTag10/Results/blured15/pointcloudID3.log",'i');

	fstream output;
	output.open("/media/DATEN/Extended_PiTag10/Results/blured15/candledata_rot.log",ios_base::out);

	double conversion = 1;//* conversion
	int fieldnumber = 7;//rot

	vector<vector<double> > tmp;

	ser->openArray("TESTTEST");
	if(!ser->readArray("TESTTEST",&tmp)) cout << "Something Went Wrong" << endl;
	ser->closeArray();
	ser->close();

	unsigned int coutX = 0;
	unsigned int coutZ = 0;
	unsigned int rotX_steps = 9;
	unsigned int rotZ_steps = 36;
	unsigned int stepsPerDist = rotX_steps*rotZ_steps;

	unsigned int validsteps = 0;
	unsigned int falsepos = 0;

	for(unsigned int i=0; i < tmp.size();i++){
		for(unsigned int j=0; j < tmp[i].size();j++){

			if(coutZ >= rotZ_steps){
				coutX++;
				coutZ=0;
				if(falsepos > 2){
					//stop computing for this distance

				} else {
					validsteps++;
				}
			}


			if(coutX >= rotX_steps){



			}


//Lost data due to pc crash

			if(tmp[i][fieldnumber] == -666) {
				falsepos++;
			}

			coutZ++;
		}
	}



}

void prepareDataforFOV2(){
//percentage stuff
	SerializeIO *ser = new SerializeIO("/media/DATEN/Extended_PiTag10/Results/blured15/pointcloudID3.log",'i');

	fstream output;
	output.open("/media/DATEN/Extended_PiTag10/Results/Fov/fov_blur15_4.log",ios_base::out);

	double conversion = 1;//* conversion
	int fieldnumber = 7;//rot

	vector<vector<double> > tmp;

	ser->openArray("TESTTEST");
	if(!ser->readArray("TESTTEST",&tmp)) cout << "Something Went Wrong" << endl;
	ser->closeArray();
	ser->close();

	unsigned int coutX = 0;
	unsigned int coutZ = 0;
	unsigned int rotX_steps = 9;
	unsigned int rotZ_steps = 36;
	unsigned int stepsPerDist = rotX_steps*rotZ_steps;

	unsigned int validpos = 0;
	unsigned int falsepos = 0;

	for(unsigned int i=0; i < tmp.size();i++){
		if(tmp[i][2] == 4.0){

			if(coutZ >= rotZ_steps){

				stringstream ss;

				ss << (double)(PI/2)/9*180/PI*coutX  << " " << (double)validpos/rotZ_steps*100 << endl;
				output << ss.str();

				coutX++;
				coutZ=0;
				validpos = 0;
				falsepos = 0;
			}

			if(tmp[i][fieldnumber] == -666) {
				falsepos++;
			} else {
				validpos++;
			}

			coutZ++;
		}
	}

	stringstream ss;

	ss << (double)(PI/2)/9*180/PI*coutX  << " " << (double)validpos/rotZ_steps*100 << endl;
	output << ss.str();

	output.close();
}

void chooseErrorPoints() {

	SerializeIO *ser = new SerializeIO("/media/DATEN/Extended_PiTag10/MAP/tag1_newerror.log",'i');
	vector<vector<double> > tmp;

	ser->openArray("TESTTEST");
	if(!ser->readArray("TESTTEST",&tmp)) cout << "Something Went Wrong" << endl;
	ser->closeArray();
	ser->close();

	SerializeIO *ser_out = new SerializeIO("/media/DATEN/Extended_PiTag10/MAP/tag1_newerror_TEST.log",'o');

	std::vector<double> parse_out;
	ser_out->openArray("TESTTEST");

	for(unsigned int i=0; i < tmp.size();i++){
		if(tmp[i][2] >= 2 && tmp[i][2] <= 3){

			ser_out->writeVector(tmp[i]);
			parse_out.empty();
		}
	}

	ser_out->closeArray();
	ser_out->close();
}

// convert data to gnuplot format!!!
int main(int argc, char** argv)
{

	//candlesticks();
	//prepareDataforFOV();
	//prepareDataforFOV2();
	chooseErrorPoints();
    ros::init(argc, argv, "fiducialsvalidation");


//    while (ros::ok())
//    {
//    	std::cout << "ehoh" << std::endl;
//    	ros::spinOnce();
//    }

    return 0;
}

