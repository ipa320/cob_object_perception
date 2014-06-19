#include "cob_texture_categorization/lbp.h"
#include <string>


#include <cmath>
#include <algorithm>
#include <complex>
#include <vector>



#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#define _USE_MATH_DEFINES


lbp::lbp()
{

}


void lbp::lbp_compute(cv::Mat image_in, int radius_in, int samples_in, struct mapping *mapping, std::string mode, int *table, float *hist)
{




	float spoints[samples_in][2];
	if(false)//(radius_in == 1 && samples_in > 0 && mode == "")
	{
		int spoints[8][2] = {	{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
								{0, 1}, {1, -1}, {1, 0}, {1, 1}};
		int samples_in = 8;
		int mapping = 0;
		std::string mode = "h";
	}else if(false)//(radius_in == 1 && samples_in  <= 0)
	{

		//error wrong input arguments
	}else if(true)//(radius_in == 1 && samples_in > 0)
	{
		//float spoints[samples_in][2];
		float circle_piece = (2*M_PI)/samples_in;
		for(int i = 1; i<=samples_in; i++)
		{
			spoints[i-1][0] = -radius_in*sin((i-1)*circle_piece);
			spoints[i-1][1] = radius_in*cos((i-1)*circle_piece);
//			std::cout << spoints[i][0]<< "spoints" << i << "/0 "<<spoints[i][1] <<"spoints" << i << "/1 ";
		}
		if(mapping != NULL)
		{
			//Überprüfung des Re und Im
			if((*mapping).samples == samples_in)
			{
				//error handling
				//std::cout<<"Error: Mapping";
			}
		}
		else  // bedingung beachten ob so korrekt, überprüfen
		{
			mapping = 0;
		}


		if(mode == "")
		{
			mode = "h";
		}
	}else//(radius_in > 1)
	{
		int spoints = radius_in;
		int neighbors[spoints];
		if(mapping == NULL)
		{
				mapping = 0;
		}
		else
		{
			//mapping errorhandling
		}
		if(mode == "")
		{
			mode = "h";
		}
	}
	// Determine the dimensions of the input image.
	double rows = image_in.rows;
	double cols = image_in.cols;
//	std::cout << rows << "rows " << cols << "cols ";


//	double miny = spoints[0][0];
//	double maxy = spoints[0][0];
//	double minx = spoints[0][1];
//	double maxx = spoints[0][1];
	double miny = 1;
	double maxy = 0;
	double minx = 1;
	double maxx = 0;

	for(int i = 0; i<samples_in; i++)
	{
		if(miny > spoints[i][0]) miny = spoints[i][0];
		if(maxy < spoints[i][0]) maxy = spoints[i][0];
		if(minx > spoints[i][1]) minx = spoints[i][1];
		if(maxx < spoints[i][1]) maxx = spoints[i][1];
	}
//	std::cout << miny << "miny "<< maxy << "maxy" << minx<<"minx "<< maxx << "maxx ";

	// Block size, each LBP code is computed within a block of size bsizey*bsizex
	double z = 0;
	int bsizey = ceil(std::max(maxy, z))-floor(std::min(miny, z))+1;
	int bsizex = ceil(std::max(maxx, z))-floor(std::min(minx, z))+1;
//	std::cout<<bsizey<<"bsizey "<<bsizex<<"bsizex ";

	// Coordinates of origin (0,0) in the block
	int origy=1-floor(std::min(miny,z));
	int origx=1-floor(std::min(minx,z));
//	std::cout<<origy<<"origy "<<origx<<"origx ";

	// Minimum allowed size for the input image depends
	// on the radius of the used LBP operator.
	if(cols < bsizex || rows < bsizey)
	{
		// Errorhandling  error('Too small input image. Should be at least (2*radius+1) x (2*radius+1)');
	}

	// Calculate dx and dy;
	int dx = cols - bsizex;
	int dy = rows - bsizey;
//	std::cout << dx << "dx " << dy << "dy ";


	// Fill the center pixel matrix C.

	cv::Mat d_image(dy, dx, CV_32FC3);
	cv::Mat C;
	cv::Mat do_image(rows, cols, CV_32FC3);

	for(int i = 0; i<rows; i++)
	{
		for(int j = 0; j<cols; j++)
		{
			for(int rgb=0; rgb<3; rgb++)
			{
				float u = image_in.at<cv::Vec3b>(i,j)[rgb];
				do_image.at<cv::Vec3f>(i,j)[rgb] = u;
			}
		}
	}
//	std::cout << origx << " " << origy << std::endl;
	cv::Rect roi = cv::Rect((origx-1), (origy-1), dx, dy);
	C = image_in(roi);

	for(int i = 0; i<dy; i++)
	{
		for(int j = 0; j<dx; j++)
		{
			for(int rgb=0; rgb<3; rgb++)
			{
				float u = C.at<cv::Vec3b>(i,j)[rgb];
				d_image.at<cv::Vec3f>(i,j)[rgb] = u;
			}
		}
	}

	int bins = pow(2,samples_in);

	// Initialize the result matrix with zeros.
	cv::Mat result = cv::Mat::zeros((dy+1),(dx+1), CV_8UC3);

//	cv::Mat mat(10+dy, 10+dx, CV_8UC3);
//	mat.at<cv::Vec3b>(y,x);
//	int image_no_interpolation[10+dx][10+dy];
//	std::vector<std::vector<int> > image_no_interpolation(std::vector<int>(10+dx, std::vector<int>(10+dy)));

	cv::Mat D = cv::Mat::zeros((dy+1),(dx+1), CV_8UC3);
	//Compute the LBP code image
	for(int i = 0; i<samples_in; i++)
	{
		double y = spoints[i][0]+origy;
		double x = spoints[i][1]+origx;
		// Calculate floors, ceils and rounds for the x and y.
		int fy = floor(y);
		int cy = ceil(y);
		int ry = round(y);

		int fx = floor(x);
		int cx = ceil(x);
		int rx = round(x);
//		std::cout << rx <<"rx "<<ry<<"ry "<< fy << "fy " << cy << "cy " << fx << "fx " << cx << "cx ";



		// Check if interpolation is needed.


		if((abs(x-rx) < 1e-6) && (abs(y-ry) < 1e-6))
		{
			cv::Rect roi = cv::Rect((rx-1), (ry-1), dx, dy);
			cv::Mat N = image_in(roi);
			// Interpolation is not needed, use original datatypes
			for(int i = 0; i<dy; i++)
			{
				for(int j = 0; j<dx; j++)
				{
					for(int rgb=0;rgb<3;rgb++)
					{
						int a = N.at<cv::Vec3b>(i,j)[rgb];
						int b = C.at<cv::Vec3b>(i,j)[rgb];
						if(a >= b)
						{
							D.at<cv::Vec3b>(i,j)[rgb]=1;
						}
						else
						{
							D.at<cv::Vec3b>(i,j)[rgb]=0;
						}
					}
				}
			}

		}else{
			cv::Mat N = cv::Mat::zeros(dy,dx, CV_32FC3);
			//std::cout<<"if_2" ;
			// Interpolation needed, use double type images
			double ty = y - fy;
			double tx = x - fx;
//			std::cout << ty << "ty "<<tx<<"tx ";

			//Calculate the interpolation weights.
			float w1 = (1 - tx) * (1 - ty);
			float w2 =      tx  * (1 - ty);
			float w3 = (1 - tx) *      ty ;
			float w4 =      tx  *      ty ;

//			std::cout << w1 << "w1+" << w2 << "+" << w3 << "+" << w4 << "+";

			// Compute interpolated pixel values

			for(int i = fy; i<dy; i++)
			{
				for(int j = fx; j<dx; j++)
				{
					for(int rgb=0;rgb<3;rgb++)
						{
							float val = d_image.at<cv::Vec3f>(i,j)[rgb];
							float num = N.at<cv::Vec3f>(i,j)[rgb];
							N.at<cv::Vec3f>(i,j)[rgb] = num + w1*val;
						}
				}
			}
			for(int i = fy; i<dy; i++)
			{
				for(int j = cx; j<dx; j++)
				{
					for(int rgb=0;rgb<3;rgb++)
						{
							float val = d_image.at<cv::Vec3f>(i,j)[rgb];
							float num = N.at<cv::Vec3f>(i,j)[rgb];
							N.at<cv::Vec3f>(i,j)[rgb] = num + w2*val;
						}
				}
			}
			for(int i = cy; i<dy; i++)
			{
				for(int j = fx; j<dx; j++)
				{
					for(int rgb=0;rgb<3;rgb++)
						{
							float val = d_image.at<cv::Vec3f>(i,j)[rgb];
							float num = N.at<cv::Vec3f>(i,j)[rgb];
							N.at<cv::Vec3f>(i,j)[rgb] = num + w3*val;
						}
				}
			}
			for(int i = cy; i<dy; i++)
			{
				for(int j = cx; j<dx; j++)
				{
					for(int rgb=0;rgb<3;rgb++)
						{
							float val = d_image.at<cv::Vec3f>(i,j)[rgb];
							float num = N.at<cv::Vec3f>(i,j)[rgb];
							N.at<cv::Vec3f>(i,j)[rgb] = num + w4*val;
						}
				}
			}


//			cv::Mat N = cv::Mat::zeros(dy,dx, CV_32FC3);
//
//			roi = cv::Rect((fx-1), (fy-1), dx, dy);
//			cv::Mat n = do_image(roi);
//			N = N + w1*n;
//			float q = N.at<cv::Vec3b>(2,2)[2];
//			float q2 = w1*n.at<cv::Vec3b>(2,2)[2];
//			std::cout << q << "n!" << q2 << "n?";
//
//			roi = cv::Rect((cx-1), (fy-1), dx, dy);
//			n = do_image(roi);
//			N = N + w2*n;
//
//			roi = cv::Rect((fx-1), (cy-1), dx, dy);
//			n = do_image(roi);
//			N = N + w3*n;
//
//			roi = cv::Rect((cx-1), (cy-1), dx, dy);
//			n = do_image(roi);
//			N = N + w4*n;


			for(int i =0 ; i<dy; i++)
			{
				for(int j = 0; j<dx; j++)
				{
					for(int rgb=0;rgb<3;rgb++)
					{
						float a = N.at<cv::Vec3f>(i,j)[rgb];
						float b = d_image.at<cv::Vec3f>(i,j)[rgb];
						if(a >= b)
						{
							D.at<cv::Vec3b>(i,j)[rgb]=1;
						}
						else
						{
							D.at<cv::Vec3b>(i,j)[rgb]=0;
						}
					}
				}
			}
		}
		// Update the result matrix.
		int v = pow(2,(i));
		result = result + v*D;
	}






	//Apply mapping if it is defined
	if(mapping != NULL)
	{
		for(int i = 0; i<=dy; i++)
		{
			for(int j = 0; j<=dx; j++)
			{
				for(int rgb=0;rgb<3;rgb++)
				{
					int pos = result.at<cv::Vec3b>(i,j)[rgb];
					result.at<cv::Vec3b>(i,j)[rgb] = table[pos];
				}
			}
		}
	}

//	for(int i = 0 ; i < 100; i++)
//	{
//		std::cout << std::endl;
//		for(int j = 0; j <20; j++)
//		{
//			for(int rgb = 0; rgb<3 ; rgb++)
//			{
//				int test = result.at<cv::Vec3b>(i,j)[rgb];
//			//	std::cout << test <<"__";
//			}
//		}
//	}

	double histogram_results[(*mapping).num];
	if(mode == "h" || mode == "hist" || mode == "nh")
	{
		//  % Return with LBP histogram if mode equals 'hist'.
		//result=hist(result(:),0:(bins-1));
		int resmin = 0;// result.at<cv::Vec3b>(0,0)[0];
		int resmax = result.at<cv::Vec3b>(0,0)[0];
		for(int i = 0; i <dy; i++)
		{
			for(int j = 0; j<dx; j++)
			{
				for(int rgb=0;rgb<3;rgb++)
				{
					int val = result.at<cv::Vec3b>(i,j)[rgb];

					if(resmin > val){ resmin = val;}
					if(resmax < val){ resmax = val;}
				}
			}
		}
	//	std::cout << resmin << "min " << resmax << "max ";
		int spek = resmax - resmin;
		int bin_lenght = 1;//spek / (*mapping).num;
	//	std::cout << spek << "spek " << bin_lenght << "bin_lenght " << (*mapping).num << "mapping num ";
		//double histogram_results[(*mapping).num];
		for(int z = 0; z<(*mapping).num; z++)
		{
			for(int i = 0; i<dy; i++)
			{
				for(int j = 0; j<dx; j++)
				{
					for(int rgb=0;rgb<3;rgb++)
					{
							if(result.at<cv::Vec3b>(i,j)[rgb] >= resmin+(bin_lenght*z) && result.at<cv::Vec3b>(i,j)[rgb] < resmin+(bin_lenght*(z+1)))
							{
								histogram_results[z] = histogram_results[z] + 1;
							}
					}
				}
			}
			//  std::cout << histogram_results[z] << "--";
		}
//		if(mode == "nh")
//		{
//			  int sum = 0;
//			  for(int i = 0; i<(*mapping).num; i++)
//			  {
//				  sum = sum + histogram_results[i];
//			  }
//			  for(int i = 0; i<(*mapping).num; i++)
//			  {
//				  histogram_results[i] = histogram_results[i]/sum;
//				  // std::cout << histogram_results[i] << "--";
//
//			  }
//		}
	}
	else
	{
		//Otherwise return a matrix of unsigned integers
		if((bins-1)<=255)   // 255 = intmax('uint8')
		{
			  for(int i = 0; i<=(*mapping).num; i++)
			  {
				  if(histogram_results[i] < 0 && histogram_results[i] > -255) histogram_results[i] =histogram_results[i] * -1 ;
				  if(histogram_results[i] <0)histogram_results[i]=0;
				  if(histogram_results[i] >255)histogram_results[i]=255;
			  }
		}else if((bins-1)<=65535)   // 65535 = intmax('uint16')
		{
			  for(int i = 0; i<=(*mapping).num; i++)
			  {
				  if(histogram_results[i] < 0 && histogram_results[i] > -65535) histogram_results[i] =histogram_results[i] * -1 ;
				  if(histogram_results[i] <0)histogram_results[i]=0;
				  if(histogram_results[i] >65535)histogram_results[i]=65535;
			  }
		}else  		//uint32 4294967295
		{
			  for(int i = 0; i<=(*mapping).num; i++)
			  {
				  if(histogram_results[i] < 0 && histogram_results[i] > -4294967295) histogram_results[i] =histogram_results[i] * -1 ;
				  if(histogram_results[i] <0)histogram_results[i]=0;
				  if(histogram_results[i] >4294967295)histogram_results[i]=4294967295;
			  }
		}
	}
	for(int i = 0; i<10; i++)
	{
		hist[i] = histogram_results[i];
//		std::cout << histogram_results[i]<<"__";
	}//std::cout << "lbp_lbp"<<std::endl;
}




