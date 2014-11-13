#include "cob_texture_categorization/texture_features.h"
#include "cob_texture_categorization/color_parameter.h"
#include "cob_texture_categorization/amadasun.h"

#include <math.h>
#include <sys/time.h>


//calculates 17 color and texture features for a selected folder/database
//
//OUTPUT: Matrices for all folders/classes Parameters_xxx whereas each row
//corresponds to one image.
//   column 1: colorfulness
//   column 2: dominant color
//   column 3: secondary dominant color
//   column 4: value/lightness/brightness
//   column 5: variety of value/lightness/brightness
//   column 6: saturation
//   column 7: variety of saturation
//   column 8: average primitive size
//   column 9: number of primitives
//   column 10: strength of primitives
//   column 11: regularity of primitives
//   column 12: contrast
//   column 13: line-likeness
//   column 14: 3D roughness
//   column 15: directionality/regularity
//   column 16: lined
//   column 17: checked

struct pos
{
	int num;
	int index;
};

struct Predicate
{
	bool operator()(const pos first, const pos second)
	{
		return first.num < second.num;
	}
};

std::vector<int> sort_index(std::vector<int> &to_sort)
{
	// Sorts inputvector and returns vector of old index position

	std::vector<int> idx(to_sort.size());
	std::vector<pos> sortid(to_sort.size());
	for(size_t i=0; i<to_sort.size(); ++i)
	{
		sortid[i].num=to_sort[i];
		sortid[i].index=i;
	}
	std::sort(sortid.begin(), sortid.end(), Predicate());

	for(unsigned int i=0; i<to_sort.size();i++)
	{
		to_sort[i]=sortid[i].num;
		idx[i]=sortid[i].index;
	}

	return idx;
}
texture_features::~texture_features()
{
}
texture_features::texture_features()
{
}
//Value 8
void texture_features::size_of_primitive(std::vector <std::vector <cv::Point> > *contours, struct feature_results *results, std::vector<int> *numPixels, double *big_comp_val)
{
//		Size of Primitives -- Value 8
		//cv::Mat image, image_gray, detected_edges;
//		Resize input image
		//resize(img, image, cv::Size(), 0.2, 0.2, cv::INTER_CUBIC);
		//cvtColor( image, image_gray, CV_BGR2GRAY );
//		Get center of image
		//cv::Mat small_image = image_gray(cv::Rect(round(image_gray.cols/4), round(image_gray.rows/4),round(image_gray.cols/2), round(image_gray.rows/2)));

//		Edge detection by Canny
		//cv::Canny( small_image, detected_edges, 30, 200, 3);  //Modify Threshold to get more or less edges
//		Get contours -- clone edge_pixels cause findContours changes input
		//cv::Mat edge_pixels = detected_edges.clone();
		//std::vector <std::vector <cv::Point> > contours;
		//std::vector<cv::Vec4i> hierarchy;
		//findContours(detected_edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());


	//	CRITERIA 1: determine biggest component

//		for(int i=0;i<(*contours).size();i++)
//		{
//			numPixels.push_back((*contours)[i].size());
//		}
	//	Sort numPixels, save old index in idx
//		std::vector<int> idx=sort_index(*numPixels);
		int size = (*numPixels).size();
		double big_comp = 0.;
		if(size>=3)
		{
			big_comp = ((*numPixels)[size-1]+(*numPixels)[size-2]+(*numPixels)[size-3])/3;
		}
		else
		{
			for(int i=0;i<size;i++)
			{
				big_comp = big_comp+(*numPixels)[i];
			}
			if(size>0)
				big_comp=big_comp/size;
		}
		big_comp=0.0025*big_comp+0.9;
	//	std::cout<<std::endl<<big_comp<<"big_comp"<<std::endl;


	//	CRITERIA 2: determine average size of large/relevant components
		double avg_size=0;
		int first_point = round(size*0.75);
		for(int i=first_point;i<size;i++)
		{
			avg_size = avg_size+(*numPixels)[i];
		}
		if(size-round(size*0.75)>0)
		{
			avg_size = avg_size/(size-round(size*0.75));
			avg_size = 0.006*avg_size+0.8;
		}else avg_size=1;
	//	std::cout<<avg_size<<"avsize "<<std::endl;
		if(avg_size<1) avg_size = 1;
		if(avg_size>5) avg_size = 5;
		if(big_comp>2.5*avg_size)big_comp=big_comp/4;
		if(big_comp<1)big_comp=1;
		if(big_comp>5)big_comp=5;
		double val8 = (big_comp + avg_size)/2;
		if(val8!=val8)std::cout<< size<<"size prob"<<std::endl;
		(*results).avg_size=val8;
		*big_comp_val = big_comp;
	//	Result Value 8


}
//Value 9
void texture_features::number_of_primitives(std::vector <std::vector <cv::Point> > *contours, struct feature_results *results, cv::Mat* edge_pixels, cv::Mat *img, std::vector<int>* numPixels, std::vector<int>* idx, double *big_comp)
{
	//    Amount of primitives -- Value 9
	//    CRITERIA 1: average distance to next edge pixel:
	//        high: few small or large primitives
	//        low: many small primitives
	//    delete small/irrelevant contours
		//cv::Canny( small_image, edge_pixels, 100, 300, 3);
		//detected_edges = edge_pixels.clone();
		//findContours(detected_edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
		//numPixels.clear();
	//	Get size/position of contours
//		for(int i=0;i<contours.size();i++)
//		{
//			numPixels.push_back(contours[i].size());
//		}
	//	clear small contours

	struct timeval zeit2;
	gettimeofday(&zeit2, 0);
		for(int i=0;i<ceil((*numPixels).size()*0.8);i++)
		{
			if((*numPixels)[i]<(*big_comp)/8)
			{
				for(unsigned int j=0;j<(*contours)[(*idx)[i]].size();j++)
				{
					for(int g=0;g<3;g++)
					{
						(*edge_pixels).at<uchar>((*contours)[(*idx)[i]][j].y, (*contours)[(*idx)[i]][j].x) = 0;
					}
				}
			}
		}

	//  Invert binary image
		float dist_edge = 0;
		cv::Mat L2dist;
		for(int i=0;i<(*edge_pixels).cols;i++)
		{
			for(int j=0;j<(*edge_pixels).rows;j++)
			{
				if((*edge_pixels).at<uchar>(j,i)!=0)
				{
					(*edge_pixels).at<uchar>(j,i)=0;
				}else{
					(*edge_pixels).at<uchar>(j,i)=255;
				}
			}
		}
	//  compute dist_edge
		distanceTransform((*edge_pixels), L2dist, CV_DIST_L2, 5);

		for(int i=0;i<L2dist.rows;i++)
		{
			for(int j=0;j<L2dist.cols;j++)
			{
				float dist_val = L2dist.at<float>(i,j);
				dist_edge = dist_edge + dist_val;
			}
		}
		dist_edge = (dist_edge/(L2dist.rows*L2dist.cols));
		dist_edge = dist_edge / ((*edge_pixels).rows*(*edge_pixels).cols)*10000;
		dist_edge=-0.06*dist_edge+4.4;
		if(dist_edge<1)dist_edge = 1;
		if(dist_edge>5)dist_edge = 5;

	//	//CRITERIA 2: amount of contours
		double amt_obj = 0.007*(*numPixels).size()+1.1;
		if(amt_obj<1)amt_obj = 1;
		if(amt_obj>5)amt_obj = 5;

		double val9;
		if(amt_obj>dist_edge)
		{
			val9=amt_obj;
		}else{
			val9=dist_edge;
		}
		(*results).prim_num=val9;
	//	Result Value 9
		struct timeval zeit3;
		gettimeofday(&zeit3, 0);
//		 std::cout << zeit3.tv_sec-zeit2.tv_sec << ':' <<  zeit3.tv_usec-zeit2.tv_usec <<"intern"<< std::endl;

}
//Value10
void texture_features::strength_of_primitives(std::vector<int> *idx,std::vector <std::vector <cv::Point> > *contours, struct feature_results *results, cv::Mat *edge_pixels, cv::Mat *img, std::vector<int>* numPixels)
{
//	 Strength of primitives -- Value 10
	//	Resize input image

	cv::Mat image_resize, image;
	resize((*img), image, cv::Size(), 0.5, 0.5, cv::INTER_CUBIC);
	cvtColor( image, image_resize, CV_BGR2HSV );

	//	extract edges
	//	Edge detection by Canny
	//	Get contours
	//	determine biggest components and delete small contours
	//	Get size/postion of contours
//		numPixels.clear();
//		for(int i=0;i<contours.size();i++)
//		{
//			numPixels.push_back(contours[i].size());
//		}
	//	Sort numPixels, save old index in idx
//		std::vector<int> idx=sort_index(*numPixels);
		for(int i=0;i<(int)round((*numPixels).size()*0.75);i++)
		{
				for(unsigned int j=0;j<(*contours)[(*idx)[i]].size();j++)
				{
					(*edge_pixels).at<uchar>((*contours)[(*idx)[i]][j].y, (*contours)[(*idx)[i]][j].x) = 0;  /// Wird in vorherigen feature schon ausgeführt!?
				}
		}
	//	Get number of non-zero points
		double edge_pixels_amount=0;
		for(int i=2;i<(*edge_pixels).rows-2;i++)
		{
			for(int j=2;j<(*edge_pixels).cols-2;j++)
			{
				if((*edge_pixels).at<uint>(i,j)!=0)
				{
					edge_pixels_amount++;
				}
			}
		}

	//	calculate standard deviation of values around edge pixels in a 3x3
	//	window and add them up
		double std_window=0;
		cv::Scalar stddev;
		cv::Scalar mean;
		cv::Mat window = cvCreateMat(3,3,CV_32FC1);
		for(int i=2;i<(*edge_pixels).rows-2;i++)
		{
			for(int j=2;j<(*edge_pixels).cols-2;j++)
			{
				if((*edge_pixels).at<uint>(i,j)!=0)
				{

					int a = 0;
					int b = 0;
					for(int l=i-1;l<=i+1;l++)
					{
						for(int o=j-1;o<=j+1;o++)
						{
							float swap =image_resize.at<cv::Vec3b>(l,o)[2];
							window.at<float>(a,b)=swap/255;
							b++;
						}
						a++;
						b=0;
					}
					cv::meanStdDev(window, mean, stddev);
					std_window = std_window + stddev.val[0];
				}
			}
		}
		double val10= 30*std_window/edge_pixels_amount*(*results).avg_size/2+1; //(*results).avg_size == val8, must be computed to be used in compute primitive size
		val10 = pow(val10,2);
		if(val10>5) val10=5;
		if(val10<1) val10=1;
		if(val10!=val10)val10=0;
		(*results).prim_strength=val10;
	//	Result Value 10
}

void texture_features::compute_regularity_of_primitives(std::vector <std::vector <cv::Point> > *contours, std::vector<int>* numPixels, cv::Mat *edge_pixels, struct feature_results *results, std::vector< std::vector<double> > *centroid, std::vector<int> *idx )
{
//	Regularity/Similarity of primitives
	//  downsampling to avoid fine structures

//		resize(img, image, cv::Size(), 0.2, 0.2, cv::INTER_CUBIC);
//		cvtColor( image, image_gray, CV_BGR2GRAY );
	//	extract edges
	//	Edge detection by Canny
//		cv::Canny( image_gray, detected_edges, 10, 30, 3);  //Modify Threshold to get more or less edges  40 190
	//	Get contours
//		edge_pixels = detected_edges.clone();
//		contours.clear();
//		hierarchy.clear();
//		findContours(detected_edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());

	//	CRITERIA 1: determine biggest component
//		numPixels.clear();
//		for(int i=0;i<contours.size();i++)
//		{
//			numPixels.push_back(contours[i].size());
//		}
	//	Sort numPixels, save old index in idx
//		std::vector<int> (*idx)=sort_index((*numPixels));
		int size = (*numPixels).size();
		int big_comp=0;
		if((*numPixels).size()>=3)	// Less amount of contours
		{
			big_comp = ((*numPixels)[size-1]+(*numPixels)[size-2]+(*numPixels)[size-3])/3;
		}else
		{
			for(int i=0;i<size;i++)
			{
				big_comp = big_comp+(*numPixels)[i];
			}
			big_comp=big_comp/size;
		}
	//	std::cout <<big_comp<<"big_comp"<<std::endl;
		if(0.025*big_comp+0.9>1)
		{
			if((0.025*big_comp+0.9)<5)big_comp=0.025*big_comp+0.9;
			else big_comp =5;
		}else big_comp =1;

		for(int i=0;i<ceil((*numPixels).size()*0.9);i++)
		{
			if((*numPixels)[i]<big_comp/8)
			{
				for(unsigned int j=0;j<(*contours)[(*idx)[i]].size();j++)
				{
					for(int g=0;g<3;g++)
					{
						(*edge_pixels).at<uchar>((*contours)[(*idx)[i]][j].y, (*contours)[(*idx)[i]][j].x) = 0;
					}
				}
			}
		}

		// determine properties of different components - compare Area, EquivDiameter, Extent
		// Get contours of optimized image
//		cv::Canny( image_gray, edge_pixels,10, 30, 3);
//		cv::Mat edge_pixels_new = edge_pixels.clone();
		std::vector <std::vector <cv::Point> > cont;
		cv::Mat test = (*edge_pixels).clone();
		std::vector<cv::Vec4i> hierarchy;
		findContours(*edge_pixels, cont, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());

		std::vector<double> equivdiameter;
		std::vector<double> extent;
		std::vector<double> area;
		cv::Scalar stddev;
		cv::Scalar mean;
		(*centroid).resize((*contours).size());

		cv::Moments moment;
		double extent_val;
//		std::cout<<std::endl<<contours.size()<<"contourssize"<<std::endl;
		for(unsigned int i=0;i<(*contours).size();i++)
		{
			moment = moments( (*contours)[i], false );
			double size = (*contours)[i].size();

			(*centroid)[i].push_back(moment.m10/moment.m00);
			(*centroid)[i].push_back(moment.m01/moment.m00);
			equivdiameter.push_back(sqrt(4*moment.m00/M_PI));
			cv::Rect boundingbox = boundingRect((*contours)[i]);
			double box1 = boundingbox.width;
			double box2 = boundingbox.height;
			area.push_back(moment.m00);
			double box = box1/box2;
			extent_val = box/size;
			extent.push_back(extent_val);
		}
		double regu;
		cv::meanStdDev(area, mean, stddev);
		double std_val = stddev.val[0];
		double mean_val = mean.val[0]*3;
	//	std::cout<<std::endl<<std_val<<"area std "<<std::endl<<mean_val<<"area mean"<<std::endl;
	//	regu = (std_val/mean_val);
		regu = (std_val/mean_val);
		cv::meanStdDev(equivdiameter, mean, stddev);
		std_val = stddev.val[0];
	//	std::cout<<std_val<<"equi std"<<std::endl;
		mean_val = mean.val[0]*3;
	//	std::cout<<mean_val<<"equi mean"<<std::endl;
	//	regu = regu/(std_val/mean_val);
		regu = regu + (std_val/mean_val)/2;
		cv::meanStdDev(extent, mean, stddev);
		std_val = stddev.val[0];
	//	std::cout<<std_val<<"extend std"<<std::endl;
	//	regu = regu/(std_val/50);
		regu = regu+(std_val)*2;
		regu = 8-regu;

	//	std::cout<<regu<<"regularity"<<std::endl;
		if(regu<5)
		{
			if(regu<1)regu=1;
		}else
		{
			regu = 5;
		}
		double val11 = regu;
		(*results).prim_regularity=val11;
	//	Result Value 11
}

void texture_features::compute_contours(cv::Mat *img, int canny_op1, int canny_op2, float resize_op1, float resize_op2, bool center, std::vector<int>* numPixels, std::vector<int> *idx, cv::Mat *detected_edges, std::vector <std::vector <cv::Point> > *contours, bool sort)
{
	cv::Mat image, image_gray;
//			Resize input image
			resize(*img, image, cv::Size(), resize_op1, resize_op2, cv::INTER_CUBIC);
			cvtColor( image, image_gray, CV_BGR2GRAY );
//		Get center of image
			if(center)
			{
				image_gray = image_gray(cv::Rect(round(image_gray.cols/4), round(image_gray.rows/4),round(image_gray.cols/2), round(image_gray.rows/2)));
			}

	//		Edge detection by Canny
			cv::Canny( image_gray, *detected_edges, canny_op1, canny_op2, 3);  //Modify Threshold to get more or less edges
	//		Get contours -- clone edge_pixels cause findContours changes input
			cv::Mat edge_pixels = (*detected_edges).clone();

			std::vector<cv::Vec4i> hierarchy;
			findContours(edge_pixels, (*contours), hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());

			for(unsigned int i=0;i<(*contours).size();i++)
			{
				(*numPixels).push_back((*contours)[i].size());
			}
			if(sort)
			{
				(*idx) = sort_index((*numPixels));
			}
}


void texture_features::linelikeness_of_primitives(cv::Mat image, std::vector <std::vector <cv::Point> > *contours, cv::Mat *edge_pixels, struct feature_results *results, std::vector<cv::RotatedRect> *ellipse_ecc, std::vector<double> *eccentricity)
{


	//	13 Line-Likeness of primitives
	//	 --> following steps already done above at Regularity/Similarity of primitives
	//
	//	      Criteria 1: search large circles/blobs by generalized Hough Transform

		std::vector<cv::Vec3f> circles_s, circles_m, circles_l, circles_xl;
		cv::Mat image_gray, img;
		resize(image, img, cv::Size(), 0.2, 0.2, cv::INTER_CUBIC);
		cvtColor( img, image_gray, CV_BGR2GRAY );

//		cv::Canny( image_gray, edge_pixels,30, 90, 3);
//		std::vector <std::vector <cv::Point> > circle_contours;
//		findContours(edge_pixels, circle_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
		struct timeval zeit;
		gettimeofday(&zeit, 0);
		cv::HoughCircles((image_gray), circles_s,CV_HOUGH_GRADIENT,1.5,10,50,150, 4, 10 );
		cv::HoughCircles((image_gray), circles_m,CV_HOUGH_GRADIENT,1,25,10,30, 10, 25 );
		cv::HoughCircles((image_gray), circles_l,CV_HOUGH_GRADIENT,1.4,50,30,90, 25, 50 );
		cv::HoughCircles((image_gray), circles_xl,CV_HOUGH_GRADIENT,1.2,110,30,90, 50, 110 );
		struct timeval zeit1;
		gettimeofday(&zeit1, 0);

//		 std::cout << zeit1.tv_sec-zeit.tv_sec << ':' <<  zeit1.tv_usec-zeit.tv_usec <<"problemfkt"<< std::endl;
	//	Code to draw circles/ellipse
	//	std::cout<<circles_s.size()<<"circ_s ";
	//	std::cout<<circles_m.size()<<"circ_m ";
	//	std::cout<<circles_l.size()<<"circ_l ";
	//	std::cout<<circles_xl.size()<<"circ_xl ";

	//	for(int i = 0;i<circles_s.size();i++)
	//	{
	////		image_gray.at<uchar>(circles_s[i][1],circles_s[i][0])=255;
	////		image_gray.at<uchar>(circles_s[i][1]+1,circles_s[i][0])=255;
	////		image_gray.at<uchar>(circles_s[i][1],circles_s[i][0]+1)=255;
	////		image_gray.at<uchar>(circles_s[i][1]-1,circles_s[i][0])=255;
	////		image_gray.at<uchar>(circles_s[i][1],circles_s[i][0]-1)=255;
	//        cv::Point center(cvRound(circles_s[i][0]), cvRound(circles_s[i][1]));
	//        int radius = cvRound(circles_s[i][2]);
	//        // draw the circle center
	//        cv::circle( image_gray, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
	//        // draw the circle outline
	//        cv::circle( image_gray, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
	//	}
	//	for(int i = 0;i<circles_m.size();i++)
	//	{
	//        cv::Point center(cvRound(circles_m[i][0]), cvRound(circles_m[i][1]));
	//        int radius = cvRound(circles_m[i][2]);
	//        // draw the circle center
	//        cv::circle( image_gray, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
	//        // draw the circle outline
	//        cv::circle( image_gray, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
	//	}
	//	for(int i = 0;i<circles_l.size();i++)
	//	{
	//        cv::Point center(cvRound(circles_l[i][0]), cvRound(circles_l[i][1]));
	//        int radius = cvRound(circles_l[i][2]);
	//        // draw the circle center
	//        cv::circle( image_gray, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
	//        // draw the circle outline
	//        cv::circle( image_gray, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
	//	}
	//	for(int i = 0;i<circles_xl.size();i++)
	//	{
	//        cv::Point center(cvRound(circles_xl[i][0]), cvRound(circles_xl[i][1]));
	//        int radius = cvRound(circles_xl[i][2]);
	//        // draw the circle center
	//        cv::circle( image_gray, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
	//        // draw the circle outline
	//        cv::circle( image_gray, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
	//	}



		double crit1 = (circles_s.size()+circles_m.size()+circles_l.size());
		if(crit1>6)
			{
				crit1 = 4-(crit1/7);
			}
		else{

			cv::Mat image_gray_small, check_lines;
			std::vector<cv::Vec2f> lines;
			cv::Canny((image_gray), check_lines,100, 300, 3);
			std::vector < std::vector<cv::Point> > lines_contours;
			std::vector<cv::Vec4i> hierarchy;
			findContours(check_lines, lines_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
			HoughLines(check_lines, lines, 1, CV_PI/180, 100, 0, 0 );
			double line_num = lines.size();
			double line_contours_num = lines_contours.size();
			double line_value = line_num/line_contours_num;
			if(line_value >1)crit1=5;
			else if(line_value>0)crit1=4;
			else crit1=1;
		}
		if(crit1<1)crit1=1;
		if(crit1>5)crit1=5;


		double count_orient=0;

		for(unsigned int i=0;i<(*contours).size();i++)
		{
			if((*contours)[i].size()>5)
				{
					(*ellipse_ecc).push_back(fitEllipse((*contours)[i]));
				}
		}
		cv::RNG rng(12345);
		  cv::Mat drawing = cv::Mat::zeros( (image_gray).size(), CV_8UC3 );
		 for(unsigned int i = 0; i< (*ellipse_ecc).size(); i++ )
		     {
		       cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
	//	       // contour
	//	       cv::drawContours( edge_pixels, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
		       // ellipse
		   	double val;
				if((*ellipse_ecc)[i].size.height>(*ellipse_ecc)[i].size.width)
				{
					val = sqrt(1-((*ellipse_ecc)[i].size.width/(*ellipse_ecc)[i].size.height));


				}else{
					val = sqrt(1-((*ellipse_ecc)[i].size.height/(*ellipse_ecc)[i].size.width));

		//			if(val<0.1)count_orient++;
				}

		       if(val<0.5 && (*ellipse_ecc)[i].size.height>20)
		       {
		       ellipse( (*edge_pixels), (*ellipse_ecc)[i], color, 2, 8 );
		       }
		       // rotated rectangle
	//	       cv::Point2f rect_points[4]; minRect[i].points( rect_points );
	//	       for( int j = 0; j < 4; j++ )
	//	          line( edge_pixels, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
		     }

	//	cv::imshow("ellipse", edge_pixels);

	//	std::cout<<ellipse_ecc.size()<<"ellipse size"<<std::endl;
		for(unsigned int i=0;i<(*ellipse_ecc).size();i++)
		{
			double val;
			if((*ellipse_ecc)[i].size.height>(*ellipse_ecc)[i].size.width)
			{
				val = sqrt(1-((*ellipse_ecc)[i].size.width/(*ellipse_ecc)[i].size.height));
				(*eccentricity).push_back(val);

			}else{
				val = sqrt(1-((*ellipse_ecc)[i].size.height/(*ellipse_ecc)[i].size.width));
				(*eccentricity).push_back(val);
	//			if(val<0.1)count_orient++;
			}
			if(val<0.6)count_orient++;
		}
//		std::cout<<count_orient<<"oriten"<<std::endl;
//		double small_lineblob = count_orient/(*ellipse_ecc).size();
//		// rescale
//		double crit2;
//		std::cout<<small_lineblob<<"smallline "<<std::endl;
//		if((small_lineblob-0.4)*10<5)
//		{
//			if(((small_lineblob-0.4)*10)<1)
//			{
//				crit2=1;
//			}else
//			{
//				crit2=(small_lineblob-0.4)*10;
//			}
//		}else{
//			crit2=5;
//		}
//		std::cout<<crit2<<"crit2 "<<std::endl;
//		double val13;
//		if(crit1<crit2)
//		{
//			val13=crit1;
//		}else{
//			val13=crit2;
//		}
		(*results).line_likeness=crit1;
		// Result Value 13

}

void texture_features::compute_151617(cv::Mat *img,  struct feature_results *results, std::vector <std::vector <cv::Point> > *circle_contours, cv::Mat *edge_pixels, std::vector<double> *eccentricity, std::vector<cv::RotatedRect> *ellipse_ecc, std::vector< std::vector<double> > *centroid)
{
	//	Directionality/Lined/Checked -- Value 15/16/17
		cv::Mat image;
		resize(*img, image, cv::Size(), 0.2, 0.2, cv::INTER_CUBIC);
		cv::Mat grad_x, grad_y;
		cv::Mat abs_grad_x, abs_grad_y;
		cv::Mat gray, hsv, hsv_conv;
		cv::cvtColor((image), gray, CV_BGR2GRAY);
		cv::cvtColor((image), hsv_conv, CV_BGR2HSV);
		cv::Mat hsv_value((image).rows, (image).cols, CV_32FC1);
		GaussianBlur( hsv_conv, hsv_conv, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );
		for(int i=0;i<(image).rows;i++)
		{
			for(int j=0;j<(image).cols;j++)
			{
				float v_val = hsv_conv.at<cv::Vec3b>(i,j)[2];
				hsv_value.at<float>(i,j)=v_val/255;
			}
		}

	// 	Gradient X
		Sobel( hsv_value, grad_x, CV_32F, 1, 0, 5, 1, 0.5, cv::BORDER_DEFAULT);
	// 	Gradient Y
		Sobel( hsv_value, grad_y, CV_32F, 0, 1, 5, 1, 0.5, cv::BORDER_DEFAULT);
	//	Polar coordinates Theta and Rho
		cv::Mat t(grad_x.rows, grad_x.cols, CV_32F);
		cv::Mat r(grad_x.rows, grad_x.cols, CV_32F);
		float r_max=-1000;
		float rmin=1000;

		for(int i=0;i<grad_x.rows;i++)
		{
			for(int j=0;j<grad_x.cols;j++)
			{
				t.at<float>(i,j)=atan2(grad_y.at<float>(i,j),grad_x.at<float>(i,j));
				double py =grad_y.at<float>(i,j);
				double px =grad_x.at<float>(i,j);
				double powy = pow(py,2);
				double powx = pow(px,2);
				float r_val = sqrt(powy+ powx);
				r.at<float>(i,j)=r_val;
				if(r_max<r_val)r_max=r_val;
				if(rmin>r_val)rmin=r_val;
			}
		}

	//  Gradients with a small magnitude are irrelevant
	//  Modulo 180\B0
		std::vector<float> t0;
		float eps = pow(2, -52);
		std::vector<float> r_vec;

		float tmin = 10;
		float tmax =-10;

		for(int i=0;i<grad_x.rows;i++)
		{
			for(int j=0;j<grad_x.cols;j++)
			{
				if(r.at<float>(i,j)<(0.15*r_max))r.at<float>(i,j)=0;
				if(r.at<float>(i,j)>eps)
				{
					if(t.at<float>(i,j)<0)
					{
						t0.push_back(t.at<float>(i,j)+M_PI);
						if(tmin>t.at<float>(i,j))tmin=t.at<float>(i,j)+M_PI;
						if(tmax<t.at<float>(i,j))tmax=t.at<float>(i,j)+M_PI;
					}
					else {
						t0.push_back(t.at<float>(i,j));
						if(tmin>t.at<float>(i,j))tmin=t.at<float>(i,j);
						if(tmax<t.at<float>(i,j))tmax=t.at<float>(i,j);
					}
					r_vec.push_back(r.at<float>(i,j));
				}
				else if(r.at<float>(i,j)<0 && r.at<float>(i,j)*(-1)>eps)
				{
					if(t.at<float>(i,j)<0)
					{
						t0.push_back(t.at<float>(i,j)+M_PI);
						if(tmin>t.at<float>(i,j))tmin=t.at<float>(i,j)+M_PI;
						if(tmax<t.at<float>(i,j))tmax=t.at<float>(i,j)+M_PI;
					}
					else {
						t0.push_back(t.at<float>(i,j));
						if(tmin>t.at<float>(i,j))tmin=t.at<float>(i,j);
						if(tmax<t.at<float>(i,j))tmax=t.at<float>(i,j);
					}
				r_vec.push_back(r.at<float>(i,j));
				}
			}
		}

	//  Histogram of angles
	//  Amount of defined directions

		std::vector<double> nbins;
		std::vector<double> Hd(16);
		double pi_part=M_PI/16;
		for(int i=0;i<=15;i++)
		{
			nbins.push_back(pi_part*i);
		}

		for(unsigned int i=0;i<t0.size();i++)
		{
			double angle_val = t0[i]/pi_part;
			double bin_num;
			modf(angle_val, &bin_num);
			Hd[bin_num]=Hd[bin_num]+1;
		}

		for(int i=0;i<16;i++)
		{
			Hd[i]=Hd[i]/t0.size();
		}


	//  Find max in hist
		double fmx=0;
		double max_value=Hd[0];
		for(unsigned int i=1;i<Hd.size();i++)
		{
			if(max_value<Hd[i])
			{
				max_value = Hd[i];
				fmx=i;
			}
		}
	//  Check if Hd has values <> NaN
		std::vector<double> Hd_sort(Hd.size());
		double lin;
		std::vector<double> ff;
	//	Shift max into center
		if(fmx>=0)
		{
			if(fmx<7)
			{
				double shift = 7-fmx;
				for(int i=0;i<Hd.size()-shift;i++)
				{
					Hd_sort[i+shift]=Hd[i];
				}
				for(int i=0;i<shift;i++)
				{
					Hd_sort[i]=Hd[(Hd.size()-shift)+i];
				}
			}else if(fmx>8)
			{
				double shift = fmx-7;
				for(int i=0;i<Hd.size()-shift;i++)
				{
					Hd_sort[i]=Hd[(shift)+i];
				}
				for(int i=0;i<=shift;i++)
				{
					Hd_sort[(Hd.size()-shift)+i]=Hd[i];
				}
			}else{
				Hd_sort=Hd;
			}
			fmx=0;
			double max_value=Hd_sort[0];
			for(unsigned int i=1;i<Hd_sort.size();i++)
			{
				if(max_value<Hd_sort[i])
				{
					max_value = Hd_sort[i];
					fmx=i+1;
				}
			}

			for(unsigned int i=0;i<Hd_sort.size();i++)
			{
				if(i+1-fmx <0)
				{
					ff.push_back((i+1-fmx)*(-1));
				}else{
					ff.push_back(i+1-fmx);
				}
			}

			if(max_value>0.3)max_value=0.3;
			double sum_Hd_ff=0;
			for(unsigned int i=3;i<Hd_sort.size()-4;i++)
			{
				sum_Hd_ff = sum_Hd_ff + Hd_sort[i]*ff[i];
			}
			lin = (max_value/sum_Hd_ff-0.06)*100;
			if(lin<1)lin=1;
		}else
		{
			lin = 1;
		}
	//	 rescale
		if(lin>5)lin=5;
		if(lin<1)lin=1;
		(*results).lined=lin;
	//	Result Value 16

	//	Checked -- Value 17
		std::vector<double> Hd_sort_checked;
	//	Switch first and second half of hist
		for(unsigned int i=8;i<Hd_sort.size();i++)
		{
			Hd_sort_checked.push_back(Hd_sort[i]);
		}
		for(int i=0;i<8;i++)
		{
			Hd_sort_checked.push_back(Hd_sort[i]);
		}

	//	peaks in the middle
		std::vector< std::vector<double> > peaks;
		double Hd_sum=0;
		for (unsigned int i=3; i<Hd_sort_checked.size()-2;i++)
		{
			if(Hd_sort_checked[i]>Hd_sort_checked[i-1] && Hd_sort_checked[i]>Hd_sort_checked[i+1] && Hd_sort_checked[i]>0.05)
			{
				peaks.resize(peaks.size()+1);
				peaks[peaks.size()-1].push_back(i);
				peaks[peaks.size()-1].push_back(Hd_sort_checked[i]);
			}
			Hd_sum = Hd_sum + Hd_sort_checked[i]*ff[i];
		}
		double checked;
	//	compute value of checked
		if(peaks.size()==0 || lin<2)
		{
			checked=1;
		}else
		{
			cv::Mat check_x, check_y;
			for(int i=0;i<grad_x.rows;i++)
			{
				for(int j=0;j<grad_x.cols;j++)
				{
					double val_grad_x = grad_x.at<float>(i,j);
					double val_grad_y = grad_y.at<float>(i,j);
					grad_x.at<float>(i,j)=round(val_grad_x*255);
					grad_y.at<float>(i,j)=round(val_grad_y*255);
				}
			}

	//		cv::Canny( grad_y, check_y,100, 300, 3);
	//		cv::Canny( grad_x, check_x,100, 300, 3);
	//		std::vector < std::vector<cv::Point> > lines_x;
	//		std::vector < std::vector<cv::Point> > lines_y;
	//		findContours(check_y, lines_y, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
	//		findContours(check_x, lines_x, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
	//		std::cout<<lines_x.size()<<"lines_x "<<lines_y.size()<<"lines_y "<<std::endl;
			double contours_factor = 1;
			if((*circle_contours).size()<=10)contours_factor=(*circle_contours).size()/20;
			else if((*circle_contours).size()>=60) contours_factor = pow(80,2)/pow((*circle_contours).size(),2);
			double min_pks=peaks[0][0];
			if(min_pks>0.3)min_pks=0.3;
			checked=5-Hd_sum*(0.4-min_pks);
			if(checked<1)checked=1;
			checked=((checked-4.7)*21+1)*contours_factor;
			if(checked<1)checked=1;
			if(checked>5)checked=5;
		}
		(*results).checked=checked;


	//	 Directionality -- Value 15
	//	 check if placement of few coarse primitives is on circle
		double crit1;
		if((*centroid).size()<50 && (*centroid).size()>=3)
		{
	//		create matrices of equation
			std::vector< std::vector<double> > circfit_a((*centroid).size());
			cv::Mat amat=cvCreateMat((*centroid).size(), 3, CV_32FC1);
			cv::Mat bmat= cvCreateMat((*centroid).size(), 1, CV_32FC1);
			cv::Mat xmat;
			for(unsigned int i=0;i<(*centroid).size();i++)
			{
				float swap_var1 = (-((*centroid)[i][0]*(*centroid)[i][0]+(*centroid)[i][1]*(*centroid)[i][1]));
				float swap_var2 = (*centroid)[i][0];
				float swap_var3 = (*centroid)[i][1];
				if(swap_var1!=swap_var1)swap_var1=0;
				if(swap_var2!=swap_var2)swap_var2=0;
				if(swap_var3!=swap_var3)swap_var3=0;
				bmat.at<float>(i,0)= swap_var1;
				amat.at<float>(i,0)= swap_var2;
				amat.at<float>(i,1)= swap_var3;
				amat.at<float>(i,2)= 1;
			}
	//		solve equation
			solve(amat, bmat, xmat, cv::DECOMP_SVD);
			float a1 = xmat.at<float>(0,0);
			float a2 = xmat.at<float>(0,1);
			float a3 = xmat.at<float>(0,2);
			float xc = -.5*a1;
			float yc = -.5*a2;
			float R  =  sqrt((a1*a1+a2*a2)/4-a3);

			std::vector<double> th;
			double pi_part=M_PI/50;
			for(int i=0;i<=100;i++)
			{
				th.push_back(pi_part*i);
			}
			std::vector<double>xunit;
			std::vector<double>yunit;
			for (unsigned int i=0;i<th.size();i++)
			{
				xunit.push_back(R*cos(th[i])+xc);
				yunit.push_back(R*sin(th[i])+yc);
			}
			double err = 0;
			for (unsigned int i=0;i<(*centroid).size();i++)
			{
				float dist = sqrt((*edge_pixels).rows*(*edge_pixels).cols);
				for (unsigned int j=0;j<xunit.size();j++)
				{
					double sqrt_val = ((*centroid)[i][0]-xunit[j])*((*centroid)[i][0]-xunit[j])+((*centroid)[i][1]-yunit[j])*((*centroid)[i][1]-yunit[j]);
					double dist_check = sqrt(sqrt_val);
					if(dist>dist_check)dist=dist_check;
				}
				err = err+dist;
			}
			err = err/(*centroid).size();
			crit1 = 5-0.085*err;
			if(crit1 >1)crit1 = 1;
		}else
		{
			crit1 = 1;
		}



	//	check distribution of centroids over image
		double bins_xhori=(image).cols/10;
		double bins_yvert=(image).rows/10;
		std::vector<double> hist_xhori(10);
		std::vector<double> hist_yvert(10);
	//	compute hist_xhori and hist_yvert
		double xhist_val;
		double yhist_val;
		for(unsigned int i=0;i<(*centroid).size();i++)
		{
				double xdivision = ((*centroid)[i][0])/bins_xhori;
				double ydivision = ((*centroid)[i][1])/bins_yvert;

				modf(xdivision, &xhist_val);
				modf(ydivision, &yhist_val);
				hist_xhori[xhist_val]=hist_xhori[xhist_val]+1;
				hist_yvert[yhist_val]=hist_yvert[yhist_val]+1;
		}
		for(int i=0;i<10;i++)
		{
			hist_xhori[i]=hist_xhori[i]/(*centroid).size();
			hist_yvert[i]=hist_yvert[i]/(*centroid).size();
		}
	//	delete first and last position of hist
		hist_xhori.pop_back();
		hist_yvert.pop_back();
		hist_xhori[0]=hist_xhori[hist_xhori.size()-1];
		hist_yvert[0]=hist_yvert[hist_yvert.size()-1];
		hist_xhori.pop_back();
		hist_yvert.pop_back();
	//	compute crit2 value
		double crit2;
		cv::Scalar stddev;
		cv::Scalar mean;
		cv::meanStdDev(hist_xhori, mean, stddev);
		crit2=stddev.val[0];
		cv::meanStdDev(hist_yvert, mean, stddev);
		crit2=5-160*(crit2 + stddev.val[0]);
		if(crit2 < 1)crit2=1;

	//	check orientation of contours
		std::vector<double> angels((*ellipse_ecc).size());
		std::vector<double> hist_angle(13);
		int hist_point_num=0;
	//	compute hist_angle
		for(unsigned int i=0;i<(*ellipse_ecc).size();i++)
		{
			if((*eccentricity)[i]>0)
			{
				if((*ellipse_ecc)[i].angle>90)
				{
					(*ellipse_ecc)[i].angle=(*ellipse_ecc)[i].angle-180;
				}
				if((*ellipse_ecc)[i].angle>=0)
				{
					double angle_val = (*ellipse_ecc)[i].angle/15;
					double bin_num;
					modf(angle_val, &bin_num);
					hist_angle[6+bin_num]=hist_angle[6+bin_num]+1;
				}else{
					double angle_val = ((*ellipse_ecc)[i].angle/15)*(-1);
					double bin_num;
					modf(angle_val, &bin_num);
					hist_angle[bin_num]=hist_angle[bin_num]+1;
				}
				hist_point_num++;
			}
		}
	// 	normalize hist_angle
		for(unsigned int i=0;i<hist_angle.size();i++)
		{
			hist_angle[i]=hist_angle[i]/hist_point_num;
		}
	//	compute crit3 value
		cv::meanStdDev(hist_angle, mean, stddev);
		double crit3= 500*stddev.val[0]*stddev.val[0];
		if(crit3<5)
		{
			if(crit3<1)crit3=1;
		}else{
			crit3=5;
		}
		double val15 = lin;
		if(val15<checked)val15=checked;
		if(val15<crit1)val15=crit1;
		if(val15<crit2)val15=crit2;
		if(val15<crit3)val15=crit3;
	//	if(val15*0.9>=1)val15=val15*0.9;
		if(crit1+crit2+crit3+lin+checked <9)val15=1;
		(*results).direct_reg=val15;


}

void texture_features::compute_features(cv::Mat *image, struct feature_results *results)
{
	std::vector<int> numPixels;
	std::vector<int> idx;
	cv::Mat edge_pixels;

//	cv::Mat sec = image.clone();

//	std::vector <std::vector <cv::Point> > contours, conts;
//	cv::Mat image_gray;
//	cvtColor( image, image_gray, CV_BGR2GRAY );
////	texture_features::compute_contours(img, canny_op1, canny_op2, resize_op1, resize_op2, center, &numPixels, &idx, &edge_pixels, &contours);
////	conts = contours;
//	double big_comp;
////Value8
//	texture_features::compute_contours(&sec, 30, 200, 0.2, 0.2, true, &numPixels, &idx, &edge_pixels, &contours, true);
//	texture_features::size_of_primitive(&contours, results, &numPixels, &big_comp);
//////Value 9
//	numPixels.clear();
//	idx.clear();
//	contours.clear();
//	texture_features::compute_contours(&sec, 100, 300, 0.2, 0.2, true, &numPixels, &idx, &edge_pixels, &contours, false);
//	texture_features::number_of_primitives(&contours, results, &edge_pixels, &sec, &numPixels, &idx, &big_comp);
////////Value 10
//	numPixels.clear();
//	idx.clear();
//	contours.clear();
//
//	texture_features::compute_contours(&sec, 40, 200, 0.5, 0.5, false, &numPixels, &idx, &edge_pixels, &contours, true);
//	texture_features::strength_of_primitives(&idx,&contours, results, &edge_pixels, &sec, &numPixels);
//
////////Value 11
//
//	std::vector< std::vector<double> > centroid;
//	numPixels.clear();
//	idx.clear();
//	contours.clear();
//	texture_features::compute_contours(&sec, 10, 30, 0.2, 0.2, false, &numPixels, &idx, &edge_pixels, &contours,true);
//	texture_features::compute_regularity_of_primitives(&contours, &numPixels, &edge_pixels, results, &centroid, &idx);

//	////Value 12
	double d = 1, contrast_raw;
	amadasun amadasun_fkt = amadasun();
	amadasun_fkt.get_amadasun(*image, d, results, contrast_raw);
//////Value 13

//	std::vector<cv::RotatedRect> ellipse_ecc;
//	std::vector<double> eccentricity;
//	numPixels.clear();
//	idx.clear();
//	contours.clear();
//	texture_features::compute_contours(&image, 30, 90, 0.2, 0.2, true, &numPixels, &idx, &edge_pixels, &contours, true);
//	texture_features::linelikeness_of_primitives(image, &contours, &edge_pixels, results, &ellipse_ecc, &eccentricity);
//
////////Value 14
//////	// 3D roughness
////////Value 15 / 16 / 17
//	struct timeval start;
//	gettimeofday(&start, 0);
////	texture_features::compute_contours(&img, 30, 90, 0.2, 0.2, true, &numPixels, &idx, &edge_pixels, &contours);
//	texture_features::compute_151617(&image, results, &contours, &edge_pixels, &eccentricity, &ellipse_ecc, &centroid);
//	struct timeval end;
//	gettimeofday(&end, 0);
//
////	calculate color values (value 1-7)
//	struct color_vals color_results;
//	color_parameter color = color_parameter();
//	color.get_color_parameter(image, results);

//
// 	 std::cout << end.tv_sec-start.tv_sec << ':' <<  end.tv_usec-start.tv_usec << "New_fkt"<< std::endl;
// 	 std::cout << zeit4.tv_sec-zeit3.tv_sec << ':' <<  zeit4.tv_usec-zeit3.tv_usec << "param4"<< std::endl;
// 	 std::cout << zeit5.tv_sec-zeit4.tv_sec << ':' <<  zeit5.tv_usec-zeit4.tv_usec <<"param5"<< std::endl;
// 	 std::cout << zeit6.tv_sec-zeit5.tv_sec << ':' <<  zeit6.tv_usec-zeit5.tv_usec <<"param6"<<  std::endl;
// 	 std::cout << zeit7.tv_sec-zeit6.tv_sec << ':' <<  zeit7.tv_usec-zeit6.tv_usec << "param7"<< std::endl;
// 	 std::cout << zeit8.tv_sec-zeit7.tv_sec << ':' <<  zeit8.tv_usec-zeit7.tv_usec << "param8"<< std::endl;


}

void texture_features::primitive_size(cv::Mat *img, struct feature_results *results, cv::Mat* raw_features)
{

//	calculate color values (value 1-7)
//	struct color_vals color_results;
//	color_parameter color = color_parameter();
//	color.get_color_parameter(img, results);

//	Size of Primitives -- Value 8
	double avg_primitive_size_raw1, avg_primitive_size_raw2;
	cv::Mat image, image_gray, detected_edges;
//	Resize input image
//	std::cout<<(*img).size()<<"size "<<(*img).type()<<"type "<<std::endl;
	if((*img).rows>200 && (*img).cols>200)
	{
		resize((*img), image, cv::Size(), 0.2, 0.2, cv::INTER_CUBIC);
	}else{
		image = *img;
	}

//	std::cout<<img.size()<<"size "<<img.type()<<"type "<<std::endl;

	cvtColor( image, image_gray, CV_BGR2GRAY );
//	Get center of image
	cv::Mat small_image = image_gray(cv::Rect(round(image_gray.cols/4), round(image_gray.rows/4),round(image_gray.cols/2), round(image_gray.rows/2)));

//	Edge detection by Canny

	cv::Canny( small_image, detected_edges, 30, 200, 3);  //Modify Threshold to get more or less edges

//	Get contours -- clone edge_pixels cause findContours changes input
	cv::Mat edge_pixels = detected_edges.clone();
	std::vector <std::vector <cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	findContours(detected_edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());

//	CRITERIA 1: determine biggest component
	std::vector<int> numPixels;
	for (unsigned int i=0;i<contours.size();i++)
	{
		numPixels.push_back(contours[i].size());
	}
//	Sort numPixels, save old index in idx
	std::vector<int> idx=sort_index(numPixels);
	int size = numPixels.size();
	double big_comp = 0;
	if(numPixels.size()>=3)
	{
		big_comp = (numPixels[size-1]+numPixels[size-2]+numPixels[size-3])/3;
	}else
	{
		for(int i=0;i<size;i++)
		{
			big_comp = big_comp+numPixels[i];
		}
		if(size>0)big_comp=big_comp/size;
	}
	avg_primitive_size_raw1 = big_comp;
	big_comp=0.0025*big_comp+0.9;
//	std::cout<<std::endl<<big_comp<<"big_comp"<<std::endl;


//	CRITERIA 2: determine average size of large/relevant components
	double avg_size=0;
	int first_point = round(size*0.75);
	for(int i=first_point;i<size;i++)
	{
		avg_size = avg_size+numPixels[i];
	}
	if(size-round(size*0.75)!=0)
	{
		avg_size = avg_size/(size-round(size*0.75));
		avg_primitive_size_raw2 = avg_size;
		avg_size = 0.006*avg_size+0.8;
	}
	else
	{
		avg_size=1;
		avg_primitive_size_raw2 = 1;
	}
//	std::cout<<avg_size<<"avsize "<<std::endl;
	if(avg_size<1) avg_size = 1;
	if(avg_size>5) avg_size = 5;
	if(big_comp>2.5*avg_size)big_comp=big_comp/4;
	if(big_comp<1)big_comp=1;
	if(big_comp>5)big_comp=5;
	double val8 = (big_comp + avg_size)/2;
//	if(val8!=val8)std::cout<< size<<"size prob"<<std::endl;
	(*results).avg_size=val8;
//	Result Value 8




//    Amount of primitives -- Value 9
//    CRITERIA 1: average distance to next edge pixel:
//        high: few small or large primitives
//        low: many small primitives
//    delete small/irrelevant contours
	double number_primitives_raw1, number_primitives_raw2;
	cv::Canny( small_image, edge_pixels, 100, 300, 3);
	detected_edges = edge_pixels.clone();
	findContours(detected_edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
	numPixels.clear();
//	Get size/position of contours
	for (unsigned int i=0;i<contours.size();i++)
	{
		numPixels.push_back(contours[i].size());
	}
//	clear small contours
	for(int i=0;i<ceil(numPixels.size()*0.8);i++)
	{
		if(numPixels[i]<big_comp/8 &&  (unsigned int)idx[i]<contours.size())
		{
			for (unsigned int j=0;j<contours[idx[i]].size();j++)
			{
				for(int g=0;g<3;g++)
				{
					edge_pixels.at<uchar>(contours[idx[i]][j].y, contours[idx[i]][j].x) = 0;
				}
			}
		}
	}

//  Invert binary image
	float dist_edge = 0;
	cv::Mat L2dist;
	for(int i=0;i<edge_pixels.cols;i++)
	{
		for(int j=0;j<edge_pixels.rows;j++)
		{
			if(edge_pixels.at<uchar>(j,i)!=0)
			{
				edge_pixels.at<uchar>(j,i)=0;
			}else{
				edge_pixels.at<uchar>(j,i)=255;
			}
		}
	}
//  compute dist_edge
	distanceTransform(edge_pixels, L2dist, CV_DIST_L2, 5);

	for(int i=0;i<L2dist.rows;i++)
	{
		for(int j=0;j<L2dist.cols;j++)
		{
			float dist_val = L2dist.at<float>(i,j);
			dist_edge = dist_edge + dist_val;
		}
	}
	if(L2dist.rows!=0 && L2dist.cols!=0 && small_image.rows*small_image.cols!=0)
	{
	dist_edge = (dist_edge/(L2dist.rows*L2dist.cols));
	dist_edge = dist_edge / (small_image.rows*small_image.cols)*10000;
	}
	number_primitives_raw1 = dist_edge;
	dist_edge=-0.06*dist_edge+4.4;
	if(dist_edge<1)dist_edge = 1;
	if(dist_edge>5)dist_edge = 5;

//	//CRITERIA 2: amount of contours
	number_primitives_raw2 = numPixels.size();
	double amt_obj = 0.007*numPixels.size()+1.1;
	if(amt_obj<1)amt_obj = 1;
	if(amt_obj>5)amt_obj = 5;

	double val9;
	if(amt_obj>dist_edge)
	{
		val9=amt_obj;
	}else{
		val9=dist_edge;
	}
	(*results).prim_num=val9;
//	Result Value 9


//  Strength of primitives -- Value 10
//	Resize input image
	double primitive_strength_raw=0.;
	cv::Mat image_resize;
	resize((*img), image, cv::Size(), 0.5, 0.5, cv::INTER_CUBIC);
	cvtColor( image, image_resize, CV_BGR2HSV );
//	extract edges
//	Edge detection by Canny
	cvtColor( image, image_gray, CV_BGR2GRAY );
	cv::Canny( image_gray, detected_edges, 40, 200, 3);
	edge_pixels = detected_edges.clone();
//	Get contours
	findContours(detected_edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
//	determine biggest components and delete small contours
//	Get size/postion of contours
	numPixels.clear();
	for (unsigned int i=0;i<contours.size();i++)
	{
		numPixels.push_back(contours[i].size());
	}
//	Sort numPixels, save old index in idx
	idx=sort_index(numPixels);
	for(int i=0;i<round(numPixels.size()*0.75) && contours.size()>=numPixels.size();i++)
	{
			for (unsigned int j=0;j<contours[idx[i]].size();j++)
			{
				edge_pixels.at<uchar>(contours[idx[i]][j].y, contours[idx[i]][j].x) = 0;
			}
	}
//	Get number of non-zero points
	double edge_pixels_amount=0;
	for(int i=2;i<edge_pixels.rows-2;i++)
	{
		for(int j=2;j<edge_pixels.cols-2;j++)
		{
			if(edge_pixels.at<uint>(i,j)!=0)
			{
				edge_pixels_amount++;
			}
		}
	}

//	calculate standard deviation of values around edge pixels in a 3x3
//	window and add them up
	double std_window=0;
	cv::Scalar stddev;
	cv::Scalar mean;
	cv::Mat window;// = cvCreateMat(3,3,CV_32FC1);
	window = cv::Mat::zeros(3,3, CV_32FC1);
	for(int i=2;i<image_resize.rows-2;i++)
	{
		for(int j=2;j<image_resize.cols-2;j++)
		{
			if(edge_pixels.at<uint>(i,j)!=0)
			{

				int a = 0;
				int b = 0;
				for(int l=i-1;l<=i+1;l++)
				{
					for(int o=j-1;o<=j+1;o++)
					{
						float swap =image_resize.at<cv::Vec3b>(l,o)[2];                 // Warum nur kanal 2??????????????????????????????????
						window.at<float>(a,b)=swap/255;
						b++;
					}
					a++;
					b=0;
				}
				cv::meanStdDev(window, mean, stddev);
				std_window = std_window + stddev.val[0];
			}
		}
	}

	double val10=0.;
	if(edge_pixels_amount!=0 && val8!=0)
	{
		primitive_strength_raw = std_window/edge_pixels_amount;
		val10= 30*std_window/edge_pixels_amount*val8/2+1;
	}
	val10 = pow(val10,2);
	if(val10>5) val10=5;
	if(val10<1) val10=1;
	if(val10!=val10)val10=0;
	(*results).prim_strength=val10;
//	Result Value 10

//  Contrast -- Value 12
	double contrast_raw;
	double d = 1;
	amadasun amadasun_fkt2 = amadasun();
	amadasun_fkt2.get_amadasun((*img), d, results, contrast_raw);


//  Regularity/Similarity of primitives
//  downsampling to avoid fine structures
	double primitive_regularity_raw1, primitive_regularity_raw2, primitive_regularity_raw3;
	resize((*img), image, cv::Size(), 0.2, 0.2, cv::INTER_CUBIC);
	cvtColor( image, image_gray, CV_BGR2GRAY );
//	extract edges
//	Edge detection by Canny
	cv::Canny( image_gray, detected_edges, 10, 30, 3);  //Modify Threshold to get more or less edges  40 190
//	Get contours
	edge_pixels = detected_edges.clone();
	contours.clear();
	hierarchy.clear();
	findContours(detected_edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());

//	CRITERIA 1: determine biggest component
	numPixels.clear();
	for (unsigned int i=0;i<contours.size();i++)
	{
		numPixels.push_back(contours[i].size());
	}
//	Sort numPixels, save old index in idx
	idx=sort_index(numPixels);
	size = numPixels.size();
	big_comp=0;
	if(numPixels.size()>=3)	// Less amount of contours
	{
		big_comp = (numPixels[size-1]+numPixels[size-2]+numPixels[size-3])/3;
	}else
	{
		for(int i=0;i<size;i++)
		{
			big_comp = big_comp+numPixels[i];
		}
		big_comp=big_comp/size;
	}
//	std::cout <<big_comp<<"big_comp"<<std::endl;
	if(0.025*big_comp+0.9>1)
	{
		if((0.025*big_comp+0.9)<5)big_comp=0.025*big_comp+0.9;
		else big_comp =5;
	}else big_comp =1;

	for(int i=0;i<ceil(numPixels.size()*0.9);i++)
	{
		if(numPixels[i]<big_comp/8 && (unsigned int)idx[i]<contours.size())
		{
			for (unsigned int j=0;j<contours[idx[i]].size();j++)
			{
				for(int g=0;g<3;g++)
				{
					edge_pixels.at<uchar>(contours[idx[i]][j].y, contours[idx[i]][j].x) = 0;
				}
			}
		}
	}

//	determine properties of different components - compare Area, EquivDiameter, Extent
//	Get contours of optimized image

//	cv::Canny( image_gray, edge_pixels,10, 30, 3);
//	cv::Mat edge_pixels_new = edge_pixels.clone();
	findContours(edge_pixels, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());

	std::vector<double> equivdiameter;
	std::vector<double> extent;
	std::vector<double> area;
	std::vector< std::vector<double> > centroid(contours.size());
	cv::Moments moment;
	double extent_val;
//	std::cout<<std::endl<<contours.size()<<"contourssize"<<std::endl;
	for (unsigned int i=0;i<contours.size();i++)
	{
		moment = moments( contours[i], false );
		double size = contours[i].size();

		centroid[i].push_back(moment.m10/moment.m00);
		centroid[i].push_back(moment.m01/moment.m00);
		equivdiameter.push_back(sqrt(4*moment.m00/M_PI));
		cv::Rect boundingbox = boundingRect(contours[i]);
		double box1 = boundingbox.width;
		double box2 = boundingbox.height;
		area.push_back(moment.m00);
		double box = box1/box2;
		extent_val = box/size;
		extent.push_back(extent_val);
	}
	double regu;
	cv::meanStdDev(area, mean, stddev);
	double std_val = stddev.val[0];
	double mean_val = mean.val[0]*3;
//	std::cout<<std::endl<<std_val<<"area std "<<std::endl<<mean_val<<"area mean"<<std::endl;
//	regu = (std_val/mean_val);
	regu = (std_val/mean_val);
	primitive_regularity_raw1 = (std_val/mean_val);
	cv::meanStdDev(equivdiameter, mean, stddev);
	std_val = stddev.val[0];
//	std::cout<<std_val<<"equi std"<<std::endl;
	mean_val = mean.val[0]*3;
//	std::cout<<mean_val<<"equi mean"<<std::endl;
//	regu = regu/(std_val/mean_val);
	regu = regu + (std_val/mean_val)/2;
	primitive_regularity_raw2 = (std_val/mean_val);
	cv::meanStdDev(extent, mean, stddev);
	std_val = stddev.val[0];
//	std::cout<<std_val<<"extend std"<<std::endl;
//	regu = regu/(std_val/50);
	regu = regu+(std_val)*2;
	primitive_regularity_raw3 = std_val;
	regu = 8-regu;

//	std::cout<<regu<<"regularity"<<std::endl;
	if(regu<5)
	{
		if(regu<1)regu=1;
	}else
	{
		regu = 5;
	}
	double val11 = regu;
	(*results).prim_regularity=val11;
//	Result Value 11

//	13 Line-Likeness of primitives
//	 --> following steps already done above at Regularity/Similarity of primitives
//
//	      Criteria 1: search large circles/blobs by generalized Hough Transform
	double line_likeness_raw;
	std::vector<cv::Vec3f> circles_s, circles_m, circles_l, circles_xl;

	cv::Canny( image_gray, edge_pixels,30, 90, 3);
	std::vector <std::vector <cv::Point> > circle_contours;
	findContours(edge_pixels, circle_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());

	cv::HoughCircles(image_gray, circles_s,CV_HOUGH_GRADIENT,1.5,10,50,150, 4, 10 );
	cv::HoughCircles(image_gray, circles_m,CV_HOUGH_GRADIENT,1,25,10,30, 10, 25 );
	cv::HoughCircles(image_gray, circles_l,CV_HOUGH_GRADIENT,1.4,50,30,90, 25, 50 );
	cv::HoughCircles(image_gray, circles_xl,CV_HOUGH_GRADIENT,1.2,110,30,90, 50, 110 );

//	Code to draw circles/ellipse
//	std::cout<<circles_s.size()<<"circ_s ";
//	std::cout<<circles_m.size()<<"circ_m ";
//	std::cout<<circles_l.size()<<"circ_l ";
//	std::cout<<circles_xl.size()<<"circ_xl ";

//	for(int i = 0;i<circles_s.size();i++)
//	{
////		image_gray.at<uchar>(circles_s[i][1],circles_s[i][0])=255;
////		image_gray.at<uchar>(circles_s[i][1]+1,circles_s[i][0])=255;
////		image_gray.at<uchar>(circles_s[i][1],circles_s[i][0]+1)=255;
////		image_gray.at<uchar>(circles_s[i][1]-1,circles_s[i][0])=255;
////		image_gray.at<uchar>(circles_s[i][1],circles_s[i][0]-1)=255;
//        cv::Point center(cvRound(circles_s[i][0]), cvRound(circles_s[i][1]));
//        int radius = cvRound(circles_s[i][2]);
//        // draw the circle center
//        cv::circle( image_gray, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
//        // draw the circle outline
//        cv::circle( image_gray, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
//	}
//	for(int i = 0;i<circles_m.size();i++)
//	{
//        cv::Point center(cvRound(circles_m[i][0]), cvRound(circles_m[i][1]));
//        int radius = cvRound(circles_m[i][2]);
//        // draw the circle center
//        cv::circle( image_gray, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
//        // draw the circle outline
//        cv::circle( image_gray, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
//	}
//	for(int i = 0;i<circles_l.size();i++)
//	{
//        cv::Point center(cvRound(circles_l[i][0]), cvRound(circles_l[i][1]));
//        int radius = cvRound(circles_l[i][2]);
//        // draw the circle center
//        cv::circle( image_gray, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
//        // draw the circle outline
//        cv::circle( image_gray, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
//	}
//	for(int i = 0;i<circles_xl.size();i++)
//	{
//        cv::Point center(cvRound(circles_xl[i][0]), cvRound(circles_xl[i][1]));
//        int radius = cvRound(circles_xl[i][2]);
//        // draw the circle center
//        cv::circle( image_gray, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
//        // draw the circle outline
//        cv::circle( image_gray, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
//	}



	double crit1 = (circles_s.size()+circles_m.size()+circles_l.size());
	if(crit1>6)
	{
		line_likeness_raw = crit1;
		crit1 = 4-(crit1/7);
	}
	else
	{
		cv::Mat image_gray_small, check_lines;
		std::vector<cv::Vec2f> lines;
		cv::Canny( image_gray, check_lines,100, 300, 3);
		std::vector < std::vector<cv::Point> > lines_contours;
		findContours(check_lines, lines_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
		HoughLines(check_lines, lines, 1, CV_PI/180, 100, 0, 0 );
		double line_num = lines.size();
		double line_contours_num = lines_contours.size();
		double line_value = line_num/line_contours_num;
		if(line_value >1)
		{
			crit1=5;
			line_likeness_raw = 5;
		}
		else if(line_value>0)
		{
			crit1=4;
			line_likeness_raw = 4;
		}
		else
		{
			crit1=1;
			line_likeness_raw = 1;
		}
	}
	if (crit1 < 1)
		crit1 = 1;
	if (crit1 > 5)
		crit1 = 5;

	std::vector<cv::RotatedRect> ellipse_ecc;
	std::vector<double> eccentricity;
	double count_orient = 0;

	for (unsigned int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size() > 5)
		{
			ellipse_ecc.push_back(fitEllipse(contours[i]));
		}
	}
	cv::RNG rng(12345);
	cv::Mat drawing = cv::Mat::zeros((*img).size(), CV_8UC3);
	for (unsigned int i = 0; i < ellipse_ecc.size(); i++)
	{
		cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
//		// contour
//		cv::drawContours( edge_pixels, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
		// ellipse
		double val;
		if(ellipse_ecc[i].size.height>ellipse_ecc[i].size.width)
		{
			val = sqrt(1-(ellipse_ecc[i].size.width/ellipse_ecc[i].size.height));
		}
		else
		{
			val = sqrt(1-(ellipse_ecc[i].size.height/ellipse_ecc[i].size.width));
//			if(val<0.1)count_orient++;
		}
		if (val < 0.5 && ellipse_ecc[i].size.height > 20)
		{
			ellipse(edge_pixels, ellipse_ecc[i], color, 2, 8);
		}
		// rotated rectangle
//	       cv::Point2f rect_points[4]; minRect[i].points( rect_points );
//	       for( int j = 0; j < 4; j++ )
//	          line( edge_pixels, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
	     }

//	cv::imshow("ellipse", edge_pixels);

//	std::cout<<ellipse_ecc.size()<<"ellipse size"<<std::endl;
	for (unsigned int i=0;i<ellipse_ecc.size();i++)
	{
		double val;
		if(ellipse_ecc[i].size.height>ellipse_ecc[i].size.width)
		{
			val = sqrt(1-(ellipse_ecc[i].size.width/ellipse_ecc[i].size.height));
			eccentricity.push_back(val);

		}
		else
		{
			val = sqrt(1-(ellipse_ecc[i].size.height/ellipse_ecc[i].size.width));
			eccentricity.push_back(val);
//			if(val<0.1)count_orient++;
		}
		if(val<0.6)count_orient++;
	}
//	std::cout<<count_orient<<"oriten"<<std::endl;
//	double small_lineblob = count_orient/ellipse_ecc.size();
	// rescale
	double crit2;
//	std::cout<<small_lineblob<<"smallline "<<std::endl;
//	if((small_lineblob-0.4)*10<5)
//	{
//		if(((small_lineblob-0.4)*10)<1)
//		{
//			crit2=1;
//		}else
//		{
//			crit2=(small_lineblob-0.4)*10;
//		}
//	}else{
//		crit2=5;
//	}
//	std::cout<<crit2<<"crit2 "<<std::endl;
//	double val13;
//	if(crit1<crit2)
//	{
//		val13=crit1;
//	}else{
//		val13=crit2;
//	}
	(*results).line_likeness=crit1;
//	Result Value 13

//	struct timeval zeit9;
//	gettimeofday(&zeit9, 0);


	// 3d roughness - dummy implementation
	results->roughness = 1. + 0.2*results->avg_size + 0.2*results->line_likeness + 0.2*results->contrast + 0.2*results->prim_strength;


	// Directionality/Lined/Checked -- Value 15/16/17
	cv::Mat grad_x, grad_y;
	cv::Mat abs_grad_x, abs_grad_y;
	cv::Mat gray, hsv, hsv_conv;
	cv::cvtColor(image, gray, CV_BGR2GRAY);
	cv::cvtColor(image, hsv_conv, CV_BGR2HSV);
	cv::Mat hsv_value(image.rows, image.cols, CV_32FC1);
	GaussianBlur( hsv_conv, hsv_conv, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT );
	for(int i=0;i<image.rows;i++)
	{
		for(int j=0;j<image.cols;j++)
		{
			float v_val = hsv_conv.at<cv::Vec3b>(i,j)[2];
			hsv_value.at<float>(i,j)=v_val/255;
		}
	}

	// Gradient X
	Sobel( hsv_value, grad_x, CV_32F, 1, 0, 5, 1, 0.5, cv::BORDER_DEFAULT);
	// Gradient Y
	Sobel( hsv_value, grad_y, CV_32F, 0, 1, 5, 1, 0.5, cv::BORDER_DEFAULT);
	// Polar coordinates Theta and Rho
	cv::Mat t(grad_x.rows, grad_x.cols, CV_32F);
	cv::Mat r(grad_x.rows, grad_x.cols, CV_32F);
	float r_max=-1000;
	float rmin=1000;

	for(int i=0;i<grad_x.rows;i++)
	{
		for(int j=0;j<grad_x.cols;j++)
		{
			t.at<float>(i,j)=atan2(grad_y.at<float>(i,j),grad_x.at<float>(i,j));
			double py =grad_y.at<float>(i,j);
			double px =grad_x.at<float>(i,j);
			double powy = pow(py,2);
			double powx = pow(px,2);
			float r_val = sqrt(powy+ powx);
			r.at<float>(i,j)=r_val;
			if(r_max<r_val)r_max=r_val;
			if(rmin>r_val)rmin=r_val;
		}
	}

	// Gradients with a small magnitude are irrelevant
	// Modulo 180\B0
	std::vector<float> t0;
	float eps = pow(2, -52);
	std::vector<float> r_vec;

	float tmin = 10;
	float tmax =-10;

	for(int i=0;i<grad_x.rows;i++)
	{
		for(int j=0;j<grad_x.cols;j++)
		{
			if(r.at<float>(i,j)<(0.15*r_max))r.at<float>(i,j)=0;
			if(r.at<float>(i,j)>eps)
			{
				if(t.at<float>(i,j)<0)
				{
					t0.push_back(t.at<float>(i,j)+M_PI);
					if(tmin>t.at<float>(i,j))tmin=t.at<float>(i,j)+M_PI;
					if(tmax<t.at<float>(i,j))tmax=t.at<float>(i,j)+M_PI;
				}
				else
				{
					t0.push_back(t.at<float>(i,j));
					if(tmin>t.at<float>(i,j))tmin=t.at<float>(i,j);
					if(tmax<t.at<float>(i,j))tmax=t.at<float>(i,j);
				}
				r_vec.push_back(r.at<float>(i,j));
			}
			else if(r.at<float>(i,j)<0 && r.at<float>(i,j)*(-1)>eps)
			{
				if(t.at<float>(i,j)<0)
				{
					t0.push_back(t.at<float>(i,j)+M_PI);
					if(tmin>t.at<float>(i,j))tmin=t.at<float>(i,j)+M_PI;
					if(tmax<t.at<float>(i,j))tmax=t.at<float>(i,j)+M_PI;
				}
				else
				{
					t0.push_back(t.at<float>(i,j));
					if(tmin>t.at<float>(i,j))tmin=t.at<float>(i,j);
					if(tmax<t.at<float>(i,j))tmax=t.at<float>(i,j);
				}
			r_vec.push_back(r.at<float>(i,j));
			}
		}
	}

	// Histogram of angles
	// Amount of defined directions
	std::vector<double> nbins;
	std::vector<double> Hd(16);
	double pi_part=M_PI/16;
	for (int i=0;i<=15;i++)
	{
		nbins.push_back(pi_part*i);
	}
	for(std::size_t i=0;i<t0.size();i++)
	{
		double angle_val = t0[i]/pi_part;
		double bin_num;
		modf(angle_val, &bin_num);
		Hd[bin_num]=Hd[bin_num]+1;
	}
	for(int i=0;i<16;i++)
	{
		Hd[i]=Hd[i]/t0.size();
	}
	// Find max in hist
	double fmx=0;
	double max_value=Hd[0];
	for(size_t i=1;i<Hd.size();i++)
	{
		if(max_value<Hd[i])
		{
			max_value = Hd[i];
			fmx=i;
		}
	}
	// Check if Hd has values <> NaN
	std::vector<double> Hd_sort(Hd.size());
	double lin;
	double lined_raw;
	std::vector<double> ff;
	// Shift max into center
	if(fmx>=0)
	{
		if(fmx<7)
		{
			double shift = 7-fmx;
			for(int i=0;i<Hd.size()-shift;i++)
			{
				Hd_sort[i+shift]=Hd[i];
			}
			for(int i=0;i<shift;i++)
			{
				Hd_sort[i]=Hd[(Hd.size()-shift)+i];
			}
		}
		else if (fmx>8)
		{
			double shift = fmx-7;
			for(int i=0;i<Hd.size()-shift;i++)
			{
				Hd_sort[i]=Hd[(shift)+i];
			}
			for(int i=0;i<=shift;i++)
			{
				Hd_sort[(Hd.size()-shift)+i]=Hd[i];
			}
		}
		else
		{
			Hd_sort=Hd;
		}
		fmx=0;
		double max_value=Hd_sort[0];
		for(size_t i=1;i<Hd_sort.size();i++)
		{
			if(max_value<Hd_sort[i])
			{
				max_value = Hd_sort[i];
				fmx=(unsigned int)i+1;
			}
		}

		for(unsigned int i=0;i<Hd_sort.size();i++)
		{
			if (i+1-fmx <0)
			{
				ff.push_back((i+1-fmx)*(-1));
			}
			else
			{
				ff.push_back(i+1-fmx);
			}
		}

		if(max_value>0.3)
			max_value=0.3;
		double sum_Hd_ff=0;
		for (unsigned int i=3;i<Hd_sort.size()-4;i++)
		{
			sum_Hd_ff = sum_Hd_ff + Hd_sort[i]*ff[i];
		}
		lined_raw = max_value/sum_Hd_ff;
		lin = (max_value/sum_Hd_ff-0.06)*100;
		if(lin<1) lin=1;
	}
	else
	{
		lin = 1;
		lined_raw = 1;
	}
	// rescale
	if(lin>5) lin=5;
	if(lin<1) lin=1;
	(*results).lined=lin;
	// Result Value 16


	// Checked -- Value 17
	std::vector<double> Hd_sort_checked;
	// Switch first and second half of hist
	for (unsigned int i=8;i<Hd_sort.size();i++)
	{
		Hd_sort_checked.push_back(Hd_sort[i]);
	}
	for(int i=0;i<8;i++)
	{
		Hd_sort_checked.push_back(Hd_sort[i]);
	}

	// peaks in the middle
	std::vector< std::vector<double> > peaks;
	double Hd_sum=0;
	for(unsigned int i=3; i<Hd_sort_checked.size()-2;i++)
	{
		if(Hd_sort_checked[i]>Hd_sort_checked[i-1] && Hd_sort_checked[i]>Hd_sort_checked[i+1] && Hd_sort_checked[i]>0.05)
		{
			peaks.resize(peaks.size()+1);
			peaks[peaks.size()-1].push_back(i);
			peaks[peaks.size()-1].push_back(Hd_sort_checked[i]);
		}
		Hd_sum = Hd_sum + Hd_sort_checked[i]*ff[i];
	}
	double checked;
	double checked_raw;
	// compute value of checked
	if(peaks.size()==0 || lin<2)
	{
		checked = 1;
		checked_raw = 1;
	}
	else
	{
		cv::Mat check_x, check_y;
		for(int i=0;i<grad_x.rows;i++)
		{
			for(int j=0;j<grad_x.cols;j++)
			{
				double val_grad_x = grad_x.at<float>(i,j);
				double val_grad_y = grad_y.at<float>(i,j);
				grad_x.at<float>(i,j)=round(val_grad_x*255);
				grad_y.at<float>(i,j)=round(val_grad_y*255);
			}
		}

//		cv::Canny( grad_y, check_y,100, 300, 3);
//		cv::Canny( grad_x, check_x,100, 300, 3);
//		std::vector < std::vector<cv::Point> > lines_x;
//		std::vector < std::vector<cv::Point> > lines_y;
//		findContours(check_y, lines_y, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
//		findContours(check_x, lines_x, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
//		std::cout<<lines_x.size()<<"lines_x "<<lines_y.size()<<"lines_y "<<std::endl;
		double contours_factor = 1;
		if (circle_contours.size()<=10)
			contours_factor = circle_contours.size()/20;
		else if (circle_contours.size()>=60)
			contours_factor = 80.*80./((double)circle_contours.size()*circle_contours.size());
		double min_pks=peaks[0][0];
		if(min_pks>0.3)
			min_pks=0.3;
		checked = 5-Hd_sum*(0.4-min_pks);
		checked_raw = checked;
		if(checked<1)
			checked = 1;
		checked = ((checked-4.7)*21+1)*contours_factor;
		if(checked<1)checked=1;
		if(checked>5)checked=5;
	}
	(*results).checked=checked;


	// Directionality -- Value 15
	// check if placement of few coarse primitives is on circle
	double directionality_raw1=0., directionality_raw2=0., directionality_raw3=0.;
	crit1 = 0.;
	if(centroid.size()<50 && centroid.size()>=3)
	{
		// create matrices of equation
		std::vector< std::vector<double> > circfit_a(centroid.size());
		cv::Mat amat=cvCreateMat(centroid.size(), 3, CV_32FC1);
		cv::Mat bmat= cvCreateMat(centroid.size(), 1, CV_32FC1);
		cv::Mat xmat;
		for (unsigned int i=0;i<centroid.size();i++)
		{
			float swap_var1 = (-(centroid[i][0]*centroid[i][0]+centroid[i][1]*centroid[i][1]));
			float swap_var2 = centroid[i][0];
			float swap_var3 = centroid[i][1];
			if(swap_var1!=swap_var1)swap_var1=0;
			if(swap_var2!=swap_var2)swap_var2=0;
			if(swap_var3!=swap_var3)swap_var3=0;
			bmat.at<float>(i,0)= swap_var1;
			amat.at<float>(i,0)= swap_var2;
			amat.at<float>(i,1)= swap_var3;
			amat.at<float>(i,2)= 1;
		}
		// solve equation
		solve(amat, bmat, xmat, cv::DECOMP_SVD);
		float a1 = xmat.at<float>(0,0);
		float a2 = xmat.at<float>(0,1);
		float a3 = xmat.at<float>(0,2);
		float xc = -.5*a1;
		float yc = -.5*a2;
		float R  =  sqrt((a1*a1+a2*a2)/4-a3);

		std::vector<double> th;
		double pi_part=M_PI/50;
		for(int i=0;i<=100;i++)
		{
			th.push_back(pi_part*i);
		}
		std::vector<double>xunit;
		std::vector<double>yunit;
		for (unsigned int i=0; i<th.size();i++)
		{
			xunit.push_back(R*cos(th[i])+xc);
			yunit.push_back(R*sin(th[i])+yc);
		}
		double err = 0;
		for (unsigned int i=0;i<centroid.size();i++)
		{
			float dist = sqrt(edge_pixels.rows*edge_pixels.cols);
			for(unsigned int j=0;j<xunit.size();j++)
			{
				double sqrt_val = (centroid[i][0]-xunit[j])*(centroid[i][0]-xunit[j])+(centroid[i][1]-yunit[j])*(centroid[i][1]-yunit[j]);
				double dist_check = sqrt(sqrt_val);
				if(dist>dist_check)dist=dist_check;
			}
			err = err+dist;
		}
		err = err/centroid.size();
		directionality_raw1 = err;
		crit1 = 5-0.085*err;
		if(crit1 >1)crit1 = 1;
	}else
	{
		crit1 = 1;
		directionality_raw1 = 1;
	}

	//std::cout<<"point"<<std::endl;

	// check distribution of centroids over image
	double bins_xhori=edge_pixels.cols/10;
	double bins_yvert=edge_pixels.rows/10;
	std::vector<double> hist_xhori(10);
	std::vector<double> hist_yvert(10);
	double xdivision = 0;
	double ydivision = 0;
	// compute hist_xhori and hist_yvert
	for (unsigned int i=0;i<centroid.size();i++)
	{
		if(bins_xhori!=0 && bins_yvert!=0){
			xdivision = centroid[i][0]/bins_xhori;
			ydivision = centroid[i][1]/bins_yvert;
		}
			double xhist_val;
			double yhist_val;
			modf(xdivision, &xhist_val);
			modf(ydivision, &yhist_val);
			if(xhist_val<hist_xhori.size())
				hist_xhori[xhist_val]=hist_xhori[xhist_val]+1;
			if(yhist_val<hist_yvert.size())
				hist_yvert[yhist_val]=hist_yvert[yhist_val]+1;
	}
	for(int i=0;i<10;i++)
	{
		hist_xhori[i]=hist_xhori[i]/centroid.size();
		hist_yvert[i]=hist_yvert[i]/centroid.size();
	}
	//std::cout<<"point"<<std::endl;

	// delete first and last position of hist
	hist_xhori.pop_back();
	hist_yvert.pop_back();
	hist_xhori[0]=hist_xhori[hist_xhori.size()-1];
	hist_yvert[0]=hist_yvert[hist_yvert.size()-1];
	hist_xhori.pop_back();
	hist_yvert.pop_back();
	// compute crit2 value
	//std::cout<<"point"<<std::endl;
	cv::meanStdDev(hist_xhori, mean, stddev);
	crit2=stddev.val[0];
	cv::meanStdDev(hist_yvert, mean, stddev);
	directionality_raw2 = crit2 + stddev.val[0];
	crit2=5-160*(crit2 + stddev.val[0]);
	if(crit2 < 1)
		crit2=1;

	// check orientation of contours
//	std::cout<<"pointeccsize"<<ellipse_ecc.size()<<std::endl;
//	std::vector<double> angel(ellipse_ecc.size());

	std::vector<double> hist_angle(13);
//	std::cout<<"pointangels"<<std::endl;
	int hist_point_num=0;
//	std::cout<<"point2"<<std::endl;
	// compute hist_angle
//	std::cout<<"eccentricity size"<<eccentricity.size()<<"  "<<eccentricity[2]<<std::endl;
//	std::cout<<"ellipse_ecc[i]"<<ellipse_ecc.size()<<"  "<<ellipse_ecc[2].angle<<std::endl;
//	std::cout<<"point3"<<std::endl;

	for (unsigned int i=0;i<ellipse_ecc.size(); i++)
	{
		if(eccentricity[i]>1)
		{
			if(ellipse_ecc[i].angle>90)
			{
				ellipse_ecc[i].angle=ellipse_ecc[i].angle-180;
			}
			if(ellipse_ecc[i].angle>=0)
			{
				double angle_val = ellipse_ecc[i].angle/15;
				double bin_num;
				modf(angle_val, &bin_num);
				if((6+bin_num)<13)
				{
					hist_angle[6+(int)bin_num]=hist_angle[6+(int)bin_num]+1;
				}else{
					std::cout<<"memory failure"<<std::endl;
				}
			}else{
				double angle_val = (ellipse_ecc[i].angle/15)*(-1);
				double bin_num;
				modf(angle_val, &bin_num);
				if(bin_num<13)
				{
				hist_angle[(int)bin_num]=hist_angle[(int)bin_num]+1;
				}else{
					std::cout<<"memory failure2"<<std::endl;
				}
			}
			hist_point_num++;
		}

	}


	// normalize hist_angle
	for (unsigned int i=0; i<hist_angle.size(); ++i)
	{
		if(hist_point_num>0) hist_angle[i]=hist_angle[i]/hist_point_num;
	}
	// compute crit3 value
	cv::meanStdDev(hist_angle, mean, stddev);
	directionality_raw3 = stddev.val[0]*stddev.val[0];
	double crit3 = 500*stddev.val[0]*stddev.val[0];
	if(crit3<5)
	{
		if(crit3<1)
			crit3=1;
	}
	else
		crit3=5;

	double val15 = lin;
	if(val15<checked)val15=checked;
	if(val15<crit1)val15=crit1;
	if(val15<crit2)val15=crit2;
	if(val15<crit3)val15=crit3;
//	if(val15*0.9>=1)val15=val15*0.9;
	if(crit1+crit2+crit3+lin+checked <9)
		val15=1;
	(*results).direct_reg=val15;


	if (raw_features != 0)
	{
		raw_features->at<float>(0, 7) = avg_primitive_size_raw1;
		raw_features->at<float>(0, 8) = avg_primitive_size_raw2;
		raw_features->at<float>(0, 9) = number_primitives_raw1;
		raw_features->at<float>(0, 10) = number_primitives_raw2;
		raw_features->at<float>(0, 11) = primitive_strength_raw;
		raw_features->at<float>(0, 12) = primitive_regularity_raw1;
		raw_features->at<float>(0, 13) = primitive_regularity_raw2;
		raw_features->at<float>(0, 14) = primitive_regularity_raw3;
		raw_features->at<float>(0, 15) = contrast_raw;
		raw_features->at<float>(0, 16) = line_likeness_raw;
		raw_features->at<float>(0, 17) = directionality_raw1;
		raw_features->at<float>(0, 18) = directionality_raw2;
		raw_features->at<float>(0, 19) = directionality_raw3;
		raw_features->at<float>(0, 20) = lined_raw;
		raw_features->at<float>(0, 21) = checked_raw;
	}

//	struct timeval zeit8;
//	gettimeofday(&zeit8, 0);
//	 std::cout << zeit8.tv_sec-zeit9.tv_sec  << ':' <<zeit8.tv_usec-zeit9.tv_usec <<"Old_fkt"<< std::endl;
}


struct MinMaxChecker
{
	double min;
	double max;

	MinMaxChecker()
	{
		min = DBL_MAX;
		max = -DBL_MAX;
	}

	void check(double val)
	{
		min = std::min(min, val);
		max = std::max(max, val);
	}
};

void texture_features::distance_to_edge_histogram(const cv::Mat& detected_edges, const cv::Mat& mask, cv::Mat& histogram)
{
	// invert binary image
	cv::Mat edge_pixels = 255 - detected_edges;

	// compute dist_edge
	cv::Mat L2dist;
	distanceTransform(edge_pixels, L2dist, CV_DIST_L2, 5);

	// compute histogram on distances
	int channels[] = {0};
	int histogram_size[] = {13};
	float ranges_0[] = {0.f, 0.1f, 2.f, 5.f, 10.f, 15.f, 20.f, 25.f, 30.f, 35.f, 40.f, 45.f, 50.f, 10000.f};
	const float* ranges[] = {ranges_0};
	cv::calcHist(&L2dist, 1, channels, mask, histogram, 1, histogram_size, ranges, false, false);

	double sum = 0.;
	for (int i=0; i<histogram.rows; ++i)
		sum += histogram.at<float>(i);
	for (int i=0; i<histogram.rows; ++i)
		std::cout << histogram.at<float>(i)/sum << "\t";
	std::cout << std::endl;

	double acc = 0.;
	double value = 0.;
	for (int i=histogram.rows-1; i>=0; --i)
	{
		acc += histogram.at<float>(i)/sum;
		value += ranges_0[i] * histogram.at<float>(i)/sum;
	}
	std::cout << "acc=" << acc << "\tvalue=" << value/acc << std::endl;
}

#define DEBUG_OUTPUTS

// todo: working on this
void texture_features::compute_texture_features(const cv::Mat& img, struct feature_results& results, cv::Mat* raw_features)
{
	std::vector<int> numPixels;
	std::vector<int> idx;
	cv::Mat edge_pixels, small_image;

	// Size of Primitives -- Value 8
	double avg_primitive_size_raw1=0., avg_primitive_size_raw2=0.;
	cv::Mat image, image_gray, detected_edges;

	// todo: sensible?
	// Resize input image
	// std::cout<< img.size()<<"size "<< img.type()<<"type "<<std::endl;
//	if(img.rows>200 && img.cols>200)
//		resize(img, image, cv::Size(), 0.2, 0.2, cv::INTER_CUBIC);
//	else
		image = img;

#ifdef DEBUG_OUTPUTS
	cv::imshow("image", image);
#endif

	// create mask of valid image regions (if not rectangular), valid regions are black, invalid is white
	cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
	const cv::Vec3b zero(0,0,0);
	for (int v=0; v<image.rows; ++v)
		for (int u=0; u<image.cols; ++u)
			if (image.at<cv::Vec3b>(v,u) == zero)
				mask.at<uchar>(v,u) = 255;
	cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 5);
	const double number_mask_pixels = cv::sum(mask==0).val[0]/255.;

	// new: ratio of edge pixels to mask pixels
	cv::cvtColor(image, image_gray, CV_BGR2GRAY);
	// edge detection by Canny Edge
	cv::Canny(image_gray, detected_edges, 30, 200, 3);  //Modify Threshold to get more or less edges		// 30, 200, 3 or 20,60,3
	detected_edges.setTo(0, mask);

	cv::Mat inv_mask = 255 - mask;	// here the black regions are invalid, white is relevant
//	cv::Mat histogram;
//	distance_to_edge_histogram(detected_edges, inv_mask, histogram);

	// CRITERIUM 1: determine average size of large/relevant components
	// if low ratio of edge pixels --> check primitive size via contours
	edge_pixels = detected_edges.clone();
	std::vector <std::vector <cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(edge_pixels, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());

	// compute primitive area from aligned bounding box
	numPixels.clear();
	for (unsigned int i=0;i<contours.size();i++)
		numPixels.push_back(cv::minAreaRect(contours[i]).size.area());
	// Sort numPixels, save old index in idx
	idx=sort_index(numPixels);
	size_t size = numPixels.size();
	double avg_primitive_size=0.;
	int first_point = round(size*0.75);
	for(size_t i=first_point;i<size;i++)	// average
		avg_primitive_size += numPixels[i];
	avg_primitive_size = ((size-first_point)>0 ? avg_primitive_size/(double)(size-first_point) : 1.);
//	avg_primitive_size = (size>0 ? numPixels[(first_point+size-1)/2] : 1.);	// median
	avg_primitive_size_raw2 = avg_primitive_size;
	avg_primitive_size = std::max(1., std::min(5., 0.004*avg_primitive_size+0.8));

	// CRITERIUM 2: determine ratio of edge pixels to mask pixels
//	edge_pixels = detected_edges.clone();
//	// remove the 75% smallest contour edges from image
//	for(size_t i=0;i<numPixels.size(); ++i)
//		if (numPixels[i] < 0.5*avg_primitive_size_raw2)
//			for (unsigned int j=0;j<contours[idx[i]].size();j++)
//				edge_pixels.at<uchar>(contours[idx[i]][j].y, contours[idx[i]][j].x) = 0;
	const double number_edge_pixels = cv::sum(detected_edges!=0).val[0]/255.;
	const double edge_mask_pixel_ratio = (number_mask_pixels > 0 ? number_edge_pixels/number_mask_pixels : 0.);
	avg_primitive_size_raw1 = edge_mask_pixel_ratio;

#ifdef DEBUG_OUTPUTS
	static MinMaxChecker avg_primitive_size_mm1;
	avg_primitive_size_mm1.check(avg_primitive_size_raw1);
	std::cout << "edge ratio: " << avg_primitive_size_raw1 << "  (" << avg_primitive_size_mm1.min << "/" << avg_primitive_size_mm1.max << ")" << std::endl;
	static MinMaxChecker avg_primitive_size_mm2;
	avg_primitive_size_mm2.check(avg_primitive_size_raw2);
	std::cout << "avg contour size: " << avg_primitive_size_raw2 << "  (" << avg_primitive_size_mm2.min << "/" << avg_primitive_size_mm2.max << ")" << std::endl;
	cv::imshow("edges", detected_edges);
#endif
	const double prim_size_ratio_factor = std::min(1., edge_mask_pixel_ratio/0.4);
	double val8 = std::max(1., std::min(5., prim_size_ratio_factor*(5.-4.*prim_size_ratio_factor) + (1.-prim_size_ratio_factor)*avg_primitive_size));
	results.avg_size=val8;

//	// old: contour sizes only
//	cv::cvtColor(image, image_gray, CV_BGR2GRAY);
//	// Get center of image
//	small_image = image_gray;//(cv::Rect(round(image_gray.cols/4), round(image_gray.rows/4),round(image_gray.cols/2), round(image_gray.rows/2)));	// hack: why smaller?
//
//	// Edge detection by Canny Edge
//	cv::Canny(small_image, detected_edges, 30, 200, 3);  //Modify Threshold to get more or less edges
//	detected_edges.setTo(0, mask);
//
//	// Get contours -- clone edge_pixels cause findContours changes input
//#ifdef DEBUG_OUTPUTS
//	edge_pixels = detected_edges.clone();
//#endif
//	std::vector <std::vector <cv::Point> > contours;
//	std::vector<cv::Vec4i> hierarchy;
//	cv::findContours(detected_edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
//
//	cv::Mat disp = cv::Mat::zeros(detected_edges.rows, detected_edges.cols, CV_8UC3);
//	for (size_t i=0; i<contours.size(); ++i)
//	{
//		cv::drawContours(disp, contours, i, CV_RGB(rand()%255, rand()%255, rand()%255), 1);
//		cv::imshow("cont", disp);
//		cv::waitKey();
//	}
//
//	// CRITERIA 1: determine biggest component
//	numPixels.clear();
//	for (unsigned int i=0;i<contours.size();i++)
//	{
//		numPixels.push_back(contours[i].size());
//		std::cout << contours[i].size() << "\t";
//	}
//	std::cout << std::endl;
//	// Sort numPixels, save old index in idx
//	idx=sort_index(numPixels);
//	size_t size = numPixels.size();
//	double big_comp = 0.;
//	int num = 0;
//	for(std::vector<int>::reverse_iterator it = numPixels.rbegin(); it!=numPixels.rend() && num<3; ++it, ++num)
//	{
//		big_comp += *it;
//		std::cout << *it << "\t";
//	}
//	std::cout << num << std::endl;
//	if (num>0)
//		big_comp=big_comp/(double)num;
//	avg_primitive_size_raw1 = big_comp;
//	big_comp=0.0025*big_comp+0.9;
//#ifdef DEBUG_OUTPUTS
//	static MinMaxChecker avg_primitive_size_mm1;
//	avg_primitive_size_mm1.check(avg_primitive_size_raw1);
//	std::cout << "big_comp: " << avg_primitive_size_raw1 << "  (" << avg_primitive_size_mm1.min << "/" << avg_primitive_size_mm1.max << ")" << std::endl;
//#endif
//
//	// CRITERIA 2: determine average size of large/relevant components
//	double avg_size=0.;
//	int first_point = round(size*0.75);
//	for(size_t i=first_point;i<size;i++)
//		avg_size = avg_size+numPixels[i];
//	if(size-first_point!=0)
//	{
//		avg_size = avg_size/(double)(size-first_point);
//		avg_primitive_size_raw2 = avg_size;
//		avg_size = 0.006*avg_size+0.8;
//	}
//	else
//	{
//		avg_size=1;
//		avg_primitive_size_raw2 = 1;
//	}
//#ifdef DEBUG_OUTPUTS
//	static MinMaxChecker avg_primitive_size_mm2;
//	avg_primitive_size_mm2.check(avg_primitive_size_raw2);
//	std::cout << "avg size: " << avg_primitive_size_raw2 << "  (" << avg_primitive_size_mm2.min << "/" << avg_primitive_size_mm2.max << ")" << std::endl;
//	cv::imshow("edge_size_of_prim_masked", edge_pixels);
//	cv::waitKey();
//#endif
//	avg_size = std::max(1., std::min(5., avg_size));
//	if(big_comp>2.5*avg_size) big_comp=big_comp*0.25;
//	big_comp = std::max(1., std::min(5., big_comp));
//	double val8 = (big_comp + avg_size)/2;
//	results.avg_size=val8;

	// Amount of primitives -- Value 9
	double number_primitives_raw1=0., number_primitives_raw2=0.;
	const double prim_num_ratio_factor = std::min(1., edge_mask_pixel_ratio/0.35);
	number_primitives_raw1 = prim_num_ratio_factor;

	// count number of larger primitives
	double number_primitives = 0.;
	for (size_t i=0; i<numPixels.size(); ++i)
		if (avg_primitive_size_raw2>1. && numPixels[i]>0.5*avg_primitive_size_raw2)
			number_primitives += 1.;
	number_primitives_raw2 = number_primitives;
	number_primitives = 1+4*sqrt(number_primitives)*0.05;

#ifdef DEBUG_OUTPUTS
	static MinMaxChecker primitive_number_mm2;
	primitive_number_mm2.check(number_primitives_raw2);
	std::cout << "number primitives: " << number_primitives_raw2 << "  (" << primitive_number_mm2.min << "/" << primitive_number_mm2.max << ")" << std::endl;
#endif
	double val9 = std::max(1., std::min(5., prim_num_ratio_factor*(1.+4.*prim_num_ratio_factor) + (1.-prim_num_ratio_factor)*number_primitives));
	results.prim_num=val9;

//	// old:
//	//    CRITERIA 1: average distance to next edge pixel:
//	//        high: few small or large primitives
//	//        low: many small primitives
//	//    delete small/irrelevant contours
////	double number_primitives_raw1, number_primitives_raw2;
//	cv::Canny( small_image, edge_pixels, 100, 300, 3);
//	detected_edges = edge_pixels.clone();
//	findContours(detected_edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
//	numPixels.clear();
//	idx.clear();
////	Get size/position of contours
//	for (unsigned int i=0;i<contours.size();i++)
//		numPixels.push_back(contours[i].size());
//	// Sort numPixels, save old index in idx
//	idx=sort_index(numPixels);
//	// clear small contours
//	for(int i=0;i<ceil(numPixels.size()*0.8);i++)
//	{
//		if(numPixels[i]<big_comp/8 &&  (unsigned int)idx[i]<contours.size())
//		{
//			for (unsigned int j=0;j<contours[idx[i]].size();j++)
//			{
//				for(int g=0;g<3;g++)
//				{
//					edge_pixels.at<uchar>(contours[idx[i]][j].y, contours[idx[i]][j].x) = 0;
//				}
//			}
//		}
//	}
//
////  Invert binary image
//	double dist_edge = 0;
//	cv::Mat L2dist;
//	for(int i=0;i<edge_pixels.cols;i++)
//	{
//		for(int j=0;j<edge_pixels.rows;j++)
//		{
//			if(edge_pixels.at<uchar>(j,i)!=0)
//			{
//				edge_pixels.at<uchar>(j,i)=0;
//			}else{
//				edge_pixels.at<uchar>(j,i)=255;
//			}
//		}
//	}
////  compute dist_edge
//	distanceTransform(edge_pixels, L2dist, CV_DIST_L2, 5);
//
//	for(int i=0;i<L2dist.rows;i++)
//	{
//		for(int j=0;j<L2dist.cols;j++)
//		{
//			float dist_val = L2dist.at<float>(i,j);
//			dist_edge = dist_edge + dist_val;
//		}
//	}
//	if(L2dist.rows!=0 && L2dist.cols!=0 && small_image.rows*small_image.cols!=0)
//	{
//	dist_edge = (dist_edge/(L2dist.rows*L2dist.cols));
//	dist_edge = dist_edge / (small_image.rows*small_image.cols)*10000;
//	}
//	number_primitives_raw1 = dist_edge;
//	dist_edge=-0.06*dist_edge+4.4;
//	if(dist_edge<1)dist_edge = 1;
//	if(dist_edge>5)dist_edge = 5;
//
////	//CRITERIA 2: amount of contours
//	number_primitives_raw2 = numPixels.size();
//	double amt_obj = 0.007*numPixels.size()+1.1;
//	if(amt_obj<1)amt_obj = 1;
//	if(amt_obj>5)amt_obj = 5;
//
//	std::cout << "prim num: old: " << std::max(amt_obj, dist_edge) << "\tnew: " << val9 << std::endl;
//
//	double val9;
//	if(amt_obj>dist_edge)
//	{
//		val9=amt_obj;
//	}else{
//		val9=dist_edge;
//	}
//	results.prim_num=val9;
//	Result Value 9


	// Strength of primitives -- Value 10
	// Resize input image
	double primitive_strength_raw=0.;
//	cv::Mat image_hsv;
	cv::Mat image_05;//, image_gray_05;
//	if(img.rows>200 && img.cols>200)
//		resize(img, image_05, cv::Size(), 0.5, 0.5, cv::INTER_CUBIC);
//	else
		image_05 = img;
//	cvtColor(image_05, image_hsv, CV_BGR2HSV);
	// extract edges
	// edge detection by canny edge
//	cvtColor(image_05, image_gray_05, CV_BGR2GRAY);
//	cv::Canny(image_gray_05, detected_edges, 40, 200, 3);
//	// Get contours
	cv::Mat large_edges = detected_edges.clone();
//	cv::findContours(edge_pixels, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
//	determine biggest components and delete small contours
//	Get size/postion of contours
//	numPixels.clear();
//	for (unsigned int i=0;i<contours.size();i++)
//	{
//		numPixels.push_back(contours[i].size());
//	}
//	Sort numPixels, save old index in idx
//	idx=sort_index(numPixels);
	// remove the 75% smallest contour edges from image
	for(size_t i=0;i<numPixels.size() /*&& contours.size()>=numPixels.size()*/; ++i)
		if (numPixels[i] < 0.5*avg_primitive_size_raw2)
			for (unsigned int j=0;j<contours[idx[i]].size();j++)
				large_edges.at<uchar>(contours[idx[i]][j].y, contours[idx[i]][j].x) = 0;

	// calculate standard deviation of values around edge pixels in a 3x3
	// window and add them up
	double std_window=0.;
	double edge_pixels_amount = 0.;
	const int window_size = 3;
	cv::Mat window = cv::Mat::zeros(window_size, window_size, CV_32FC1);
	cv::Scalar stddev;
	cv::Scalar mean;
	for(int v=window_size/2;v<image_gray.rows-window_size/2;v++)
	{
		for(int u=window_size/2;u<image_gray.cols-window_size/2;u++)
		{
			if(large_edges.at<uint>(v,u)!=0)
			{
				int a = 0;
				int b = 0;
				for(int l=v-window_size/2; l<=v+window_size/2; ++l)
				{
					for(int o=u-window_size/2; o<=u+window_size/2; ++o)
					{
						float swap = image_gray.at<uchar>(l,o);
						window.at<float>(a,b) = swap/255.;
						b++;
					}
					a++;
					b=0;
				}
				cv::meanStdDev(window, mean, stddev);
				std_window += stddev.val[0];
				edge_pixels_amount++;
			}
		}
	}

	double val10=0.;
	if(edge_pixels_amount!=0)
	{
		primitive_strength_raw = std_window/edge_pixels_amount;
		val10 = 1. + 40*primitive_strength_raw * (val8/8.+0.375);
	}
	val10 = std::max(1., std::min(5., val10));
#ifdef DEBUG_OUTPUTS
	static MinMaxChecker primitive_strength_mm1;
	primitive_strength_mm1.check(primitive_strength_raw);
	std::cout << "primitive strength: " << primitive_strength_raw << "  (" << primitive_strength_mm1.min << "/" << primitive_strength_mm1.max << ")" << std::endl;
	cv::imshow("large edges", large_edges);
#endif
//	val10 = val10*val10;
//	if(val10>5) val10=5;
//	if(val10<1) val10=1;
//	if(val10!=val10)val10=0;
	results.prim_strength=val10;


//  Contrast -- Value 12
	double contrast_raw;
	double d = 1;
	amadasun amadasun_fkt2 = amadasun();
	amadasun_fkt2.get_amadasun(img, d, &results, contrast_raw);
	//results.contrast = std::max(1., std::min(5., 1. + 4*contrast_raw/1.5));
#ifdef DEBUG_OUTPUTS
	static MinMaxChecker contrast_mm1;
	contrast_mm1.check(contrast_raw);
	std::cout << "contrast: " << contrast_raw << "  (" << contrast_mm1.min << "/" << contrast_mm1.max << ")" << std::endl;
#endif


	// Regularity/Similarity of primitives -- Value 11
	// downsampling to avoid fine structures
	double primitive_regularity_raw1=0., primitive_regularity_raw2=0., primitive_regularity_raw3=0.;
//	//	resize(img, image, cv::Size(), 0.2, 0.2, cv::INTER_CUBIC);	// use the one from above
//	//	cvtColor( image, image_gray, CV_BGR2GRAY );
//	// extract edges
//	// Edge detection by Canny
//	cv::Canny(image_gray, detected_edges, 10, 30, 3);  //Modify Threshold to get more or less edges  40 190
//	// Get contours
//	edge_pixels = detected_edges.clone();
//	contours.clear();
//	hierarchy.clear();
//	findContours(detected_edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
//
//	// CRITERIA 1: determine biggest component
//	numPixels.clear();
//	for (unsigned int i=0;i<contours.size();i++)
//	{
//		numPixels.push_back(contours[i].size());
//	}
//	// Sort numPixels, save old index in idx
//	idx=sort_index(numPixels);
//	size = numPixels.size();
//	double big_comp=0;
//	if(numPixels.size()>=3)	// Less amount of contours
//	{
//		big_comp = (numPixels[size-1]+numPixels[size-2]+numPixels[size-3])/3;
//	}else
//	{
//		for(size_t i=0;i<size;i++)
//		{
//			big_comp = big_comp+numPixels[i];
//		}
//		big_comp=big_comp/size;
//	}
////	std::cout <<big_comp<<"big_comp"<<std::endl;
//	if(0.025*big_comp+0.9>1)
//	{
//		if((0.025*big_comp+0.9)<5)big_comp=0.025*big_comp+0.9;
//		else big_comp =5;
//	}else big_comp =1;
//
//	for(int i=0;i<ceil(numPixels.size()*0.9);i++)
//	{
//		if(numPixels[i]<big_comp/8 && (unsigned int)idx[i]<contours.size())
//		{
//			for (unsigned int j=0;j<contours[idx[i]].size();j++)
//			{
//				for(int g=0;g<3;g++)
//				{
//					edge_pixels.at<uchar>(contours[idx[i]][j].y, contours[idx[i]][j].x) = 0;
//				}
//			}
//		}
//	}
//
//	// determine properties of different components - compare Area, EquivDiameter, Extent
//	// Get contours of optimized image
//
////	cv::Canny( image_gray, edge_pixels,10, 30, 3);
////	cv::Mat edge_pixels_new = edge_pixels.clone();
//	findContours(edge_pixels, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());

	std::vector<double> equivdiameter;
	std::vector<double> extent;
	std::vector<double> area;
	for (size_t i=0; i<contours.size(); ++i)
	{
		double size = contours[i].size();
		equivdiameter.push_back(sqrt(4*size/M_PI));
		cv::Rect boundingbox = cv::boundingRect(contours[i]);
		double box = (double)boundingbox.width*(double)boundingbox.height;
		extent.push_back(box/size);
		area.push_back(size);
	}
	double regu = 0.;
	if (contours.size()>0)
	{
		cv::meanStdDev(area, mean, stddev);
		double ratio = (mean.val[0]>0 ? stddev.val[0]/mean.val[0] : 0);
		regu = ratio * 0.2;// * 1./3.;
		primitive_regularity_raw1 = ratio;
		// std::cout << "area mean=" << mean.val[0] << "\tstd=" << stddev.val[0] << std::endl;
		cv::meanStdDev(equivdiameter, mean, stddev);
		ratio = (mean.val[0]>0 ? stddev.val[0]/mean.val[0] : 0);
		regu += ratio * 0.25;// * 1./6.;
		primitive_regularity_raw2 = ratio;
		// std::cout << "equivdiameter mean=" << mean.val[0] << "\tstd=" << stddev.val[0] << std::endl;
		cv::meanStdDev(extent, mean, stddev);
		ratio = (mean.val[0]>0 ? stddev.val[0]/mean.val[0] : 0);
		regu += ratio;// * 2./3.;
		primitive_regularity_raw3 = ratio;
	}

	double val11 = std::max(1., std::min(5., 5. - 4./3.*regu));
	results.prim_regularity=val11;
#ifdef DEBUG_OUTPUTS
	static MinMaxChecker primitive_regularity_mm1;
	primitive_regularity_mm1.check(primitive_regularity_raw1);
	std::cout << "primitive regularity 1: " << primitive_regularity_raw1 << "  (" << primitive_regularity_mm1.min << "/" << primitive_regularity_mm1.max << ")" << std::endl;
	static MinMaxChecker primitive_regularity_mm2;
	primitive_regularity_mm2.check(primitive_regularity_raw2);
	std::cout << "primitive regularity 2: " << primitive_regularity_raw2 << "  (" << primitive_regularity_mm2.min << "/" << primitive_regularity_mm2.max << ")" << std::endl;
	static MinMaxChecker primitive_regularity_mm3;
	primitive_regularity_mm3.check(primitive_regularity_raw3);
	std::cout << "primitive regularity 3: " << primitive_regularity_raw3 << "  (" << primitive_regularity_mm3.min << "/" << primitive_regularity_mm3.max << ")" << std::endl;
#endif


//	13 Line-Likeness of primitives
//	 --> following steps already done above at Regularity/Similarity of primitives
//
//	      Criteria 1: search large circles/blobs by generalized Hough Transform
	double line_likeness_raw = 0.;
	std::vector<cv::Vec3f> circles_s, circles_m, circles_l, circles_xl;

	cv::Canny(image_gray, edge_pixels, 30, 90, 3);
	std::vector <std::vector <cv::Point> > circle_contours;
	findContours(edge_pixels, circle_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());

	double crit1 = 0.;
//	cv::HoughCircles(image_gray, circles_s, CV_HOUGH_GRADIENT, 1.5,  10,  100, 15,   4,  10);
//	cv::HoughCircles(image_gray, circles_m, CV_HOUGH_GRADIENT, 1,    25,  100, 30,  10,  25);
//	cv::HoughCircles(image_gray, circles_l, CV_HOUGH_GRADIENT, 1.4,  50,  100, 90,  25,  50);
//	cv::HoughCircles(image_gray, circles_xl, CV_HOUGH_GRADIENT, 1.2, 110, 100, 90,  50, 110);
//
////	Code to draw circles/ellipse
////	std::cout<<circles_s.size()<<"circ_s ";
////	std::cout<<circles_m.size()<<"circ_m ";
////	std::cout<<circles_l.size()<<"circ_l ";
////	std::cout<<circles_xl.size()<<"circ_xl ";
//
////	for(int i = 0;i<circles_s.size();i++)
////	{
//////		image_gray.at<uchar>(circles_s[i][1],circles_s[i][0])=255;
//////		image_gray.at<uchar>(circles_s[i][1]+1,circles_s[i][0])=255;
//////		image_gray.at<uchar>(circles_s[i][1],circles_s[i][0]+1)=255;
//////		image_gray.at<uchar>(circles_s[i][1]-1,circles_s[i][0])=255;
//////		image_gray.at<uchar>(circles_s[i][1],circles_s[i][0]-1)=255;
////        cv::Point center(cvRound(circles_s[i][0]), cvRound(circles_s[i][1]));
////        int radius = cvRound(circles_s[i][2]);
////        // draw the circle center
////        cv::circle( image_gray, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
////        // draw the circle outline
////        cv::circle( image_gray, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
////	}
////	for(int i = 0;i<circles_m.size();i++)
////	{
////        cv::Point center(cvRound(circles_m[i][0]), cvRound(circles_m[i][1]));
////        int radius = cvRound(circles_m[i][2]);
////        // draw the circle center
////        cv::circle( image_gray, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
////        // draw the circle outline
////        cv::circle( image_gray, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
////	}
////	for(int i = 0;i<circles_l.size();i++)
////	{
////        cv::Point center(cvRound(circles_l[i][0]), cvRound(circles_l[i][1]));
////        int radius = cvRound(circles_l[i][2]);
////        // draw the circle center
////        cv::circle( image_gray, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
////        // draw the circle outline
////        cv::circle( image_gray, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
////	}
////	for(int i = 0;i<circles_xl.size();i++)
////	{
////        cv::Point center(cvRound(circles_xl[i][0]), cvRound(circles_xl[i][1]));
////        int radius = cvRound(circles_xl[i][2]);
////        // draw the circle center
////        cv::circle( image_gray, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
////        // draw the circle outline
////        cv::circle( image_gray, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
////	}
//
//	crit1 = circles_s.size()+circles_m.size()+circles_l.size()+circles_xl.size();
//	if(crit1>6)
//	{
//		line_likeness_raw = crit1;
//		crit1 = 4-(crit1/7);
//	}
//	else
//	{
//		cv::Mat image_gray_small, check_lines;
//		std::vector<cv::Vec2f> lines;
//		cv::Canny(image_gray, check_lines, 100, 300, 3);
//		std::vector < std::vector<cv::Point> > lines_contours;
//		cv::findContours(check_lines, lines_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
//		cv::HoughLines(check_lines, lines, 1, CV_PI/180, 100, 0, 0);
//		double line_num = lines.size();
//		double line_contours_num = lines_contours.size();
//		double line_value = line_num/line_contours_num;
//		if(line_value >1)
//		{
//			crit1=5;
//			line_likeness_raw = 5;
//		}
//		else if(line_value>0)
//		{
//			crit1=4;
//			line_likeness_raw = 4;
//		}
//		else
//		{
//			crit1=1;
//			line_likeness_raw = 1;
//		}
//	}
//	crit1 = std::max(1., std::min(5., crit1));

	std::vector<cv::RotatedRect> ellipse_ecc;
	for (unsigned int i = 0; i < contours.size(); i++)
		if (contours[i].size() > 5)
			ellipse_ecc.push_back(cv::fitEllipse(contours[i]));

	//	cv::RNG rng(12345);
//	cv::Mat drawing = cv::Mat::zeros(img.size(), CV_8UC3);
//	for (unsigned int i = 0; i < ellipse_ecc.size(); i++)
//	{
//		cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
////		// contour
////		cv::drawContours( edge_pixels, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
//		// ellipse
//		double val = sqrt(1-(std::min(ellipse_ecc[i].size.width, ellipse_ecc[i].size.height)/std::max(ellipse_ecc[i].size.width, ellipse_ecc[i].size.height)));
//		if (val < 0.5 && ellipse_ecc[i].size.height > 20)
//		{
//			cv::ellipse(edge_pixels, ellipse_ecc[i], color, 2, 8);
//		}
//	}
//	// cv::imshow("ellipse", edge_pixels);

	std::vector<double> eccentricity;
	double crit2 = 0.;
	for (unsigned int i=0;i<ellipse_ecc.size();i++)
	{
		double val = sqrt(1-(std::min(ellipse_ecc[i].size.width, ellipse_ecc[i].size.height)/std::max(ellipse_ecc[i].size.width, ellipse_ecc[i].size.height)));
		eccentricity.push_back(val);
		crit2 += val;
	}
	crit2 = (eccentricity.size()>0 ? crit2/(double)eccentricity.size() : 0.);
	line_likeness_raw = crit2;
	crit2 = std::max(0., std::min(1., (crit2-0.5)*2.5));
	results.line_likeness = std::max(1., std::min(5., 1. + 4.*crit2*crit2));//crit1;
#ifdef DEBUG_OUTPUTS
	static MinMaxChecker line_likeness_mm1;
	line_likeness_mm1.check(line_likeness_raw);
	std::cout << "line likeness: " << line_likeness_raw << "  (" << line_likeness_mm1.min << "/" << line_likeness_mm1.max << ")" << std::endl;
#endif

	// 3d roughness - dummy implementation -- Value 14
	results.roughness = 1. + 0.2*results.avg_size + 0.2*results.line_likeness + 0.2*results.contrast + 0.2*results.prim_strength;


	// Directionality/Lined/Checked -- Value 15/16/17
	double lined_raw = 0.;
	cv::Mat grad_x, grad_y;
	cv::Mat abs_grad_x, abs_grad_y;
	cv::Mat hsv_conv;
	cv::cvtColor(image, hsv_conv, CV_BGR2HSV);
	cv::Mat v_channel(image.rows, image.cols, CV_32FC1);
	GaussianBlur(hsv_conv, hsv_conv, cv::Size(3,3), 0, 0, cv::BORDER_DEFAULT);
	for(int v=0;v<image.rows;v++)
		for(int u=0;u<image.cols;u++)
			v_channel.at<float>(v,u) = (float)hsv_conv.at<cv::Vec3b>(v,u)[2]/255.f;

	// gradient X
	cv::Sobel(v_channel, grad_x, CV_32F, 1, 0, 5, 1, 0., cv::BORDER_DEFAULT);
	// gradient Y
	cv::Sobel(v_channel, grad_y, CV_32F, 0, 1, 5, 1, 0., cv::BORDER_DEFAULT);

	// polar coordinates Theta and Rho
	cv::Mat theta(grad_x.rows, grad_x.cols, CV_32F);
	cv::Mat magnitude(grad_x.rows, grad_x.cols, CV_32F);
	double magnitude_mean = 0.;
	for(int v=0;v<grad_x.rows;v++)
	{
		for(int u=0;u<grad_x.cols;u++)
		{
			const double px = grad_x.at<float>(v,u);
			const double py = grad_y.at<float>(v,u);
			theta.at<float>(v,u) = atan2(py, px);
			float mag_val = sqrt(py*py + px*px);
			magnitude.at<float>(v,u) = mag_val;
			if (inv_mask.at<uchar>(v,u) != 0)
				magnitude_mean += mag_val;
		}
	}
	const int number_inv_mask_pixels = cv::sum(inv_mask!=0).val[0]/255.;
	magnitude_mean /= (number_inv_mask_pixels > 0 ? (double)number_inv_mask_pixels : 1.);

	// create mask for relevant gradients (strong enough, above noise level)
	cv::Mat magnitude_mask = inv_mask.clone();
	for(int v=0;v<magnitude.rows;v++)
		for(int u=0;u<magnitude.cols;u++)
			if(magnitude.at<float>(v,u) < (0.5*magnitude_mean))
				magnitude_mask.at<uchar>(v,u)=0;
//	const int number_magnitude_mask_pixels = cv::sum(magnitude_mask!=0).val[0]/255.;
//	// todo: use this ratio?
//	double gradient_pixel_ratio = (double)number_magnitude_mask_pixels/(double)number_inv_mask_pixels;
//	std::cout << "gradient_pixel_ratio=" << gradient_pixel_ratio << std::endl;

	// map gradient directions to [0, M_PI] range
	for(int v=0;v<theta.rows;v++)
		for(int u=0;u<theta.cols;u++)
			if(theta.at<float>(v,u)<0)
				theta.at<float>(v,u) += M_PI;

	// compute histogram on directions
	int channels[] = {0};
	int histogram_width = 18;
	int histogram_size[] = {histogram_width};
	float ranges_0[] = {0.f, M_PI};
	const float* ranges[] = {ranges_0};
	cv::Mat histogram = cv::Mat::ones(histogram_width, 1, CV_32FC1);
	cv::calcHist(&theta, 1, channels, magnitude_mask, histogram, 1, histogram_size, ranges, true, true);

	//std::cout << "gradients:\n";
	double sum = 0.;
	for (int i=0; i<histogram.rows; ++i)
		sum += histogram.at<float>(i);
	for (int i=0; i<histogram.rows; ++i)
	{
		histogram.at<float>(i) /= sum;
		//std::cout << histogram.at<float>(i) << "\t";
	}
	//std::cout << std::endl;

	// find maximum bin, add neighboring bins
	double min_val=0., max_val1=0.;
	cv::Point min_p, max_p1;
	cv::minMaxLoc(histogram, &min_val, &max_val1, &min_p, &max_p1);
	if (max_p1.y > 0)
		max_val1 += histogram.at<float>(max_p1.y-1);
	else
		max_val1 += histogram.at<float>(histogram.rows-1);
	max_val1 += histogram.at<float>((max_p1.y+1)%histogram.rows);

	lined_raw = max_val1;
	results.lined = std::max(1., std::min(5., 1. + 4*3.3*(max_val1-0.4)));

#ifdef DEBUG_OUTPUTS
	static MinMaxChecker lined_mm1;
	lined_mm1.check(lined_raw);
	std::cout << "lined: " << lined_raw << "  (" << lined_mm1.min << "/" << lined_mm1.max << ")" << std::endl;
#endif

//	// Gradients with a small magnitude are irrelevant
//	// Modulo 180\B0
//	std::vector<float> t0;
//	float eps = pow(2, -52);
//
//	float tmin = 10;
//	float tmax =-10;
//
//	for(int i=0;i<grad_x.rows;i++)
//	{
//		for(int j=0;j<grad_x.cols;j++)
//		{
//			if(magnitude.at<float>(i,j) > (0.15*magnitude_mean))
//				magnitude.at<float>(i,j)=0;
//			if(magnitude.at<float>(i,j)>eps)
//			{
//				if(theta.at<float>(i,j)<0)
//				{
//					t0.push_back(theta.at<float>(i,j)+M_PI);
//					if(tmin>theta.at<float>(i,j))tmin=theta.at<float>(i,j)+M_PI;
//					if(tmax<theta.at<float>(i,j))tmax=theta.at<float>(i,j)+M_PI;
//				}
//				else
//				{
//					t0.push_back(theta.at<float>(i,j));
//					if(tmin>theta.at<float>(i,j))tmin=theta.at<float>(i,j);
//					if(tmax<theta.at<float>(i,j))tmax=theta.at<float>(i,j);
//				}
//			}
//			else if(magnitude.at<float>(i,j)<0 && magnitude.at<float>(i,j)*(-1)>eps)
//			{
//				if(theta.at<float>(i,j)<0)
//				{
//					t0.push_back(theta.at<float>(i,j)+M_PI);
//					if(tmin>theta.at<float>(i,j))tmin=theta.at<float>(i,j)+M_PI;
//					if(tmax<theta.at<float>(i,j))tmax=theta.at<float>(i,j)+M_PI;
//				}
//				else
//				{
//					t0.push_back(theta.at<float>(i,j));
//					if(tmin>theta.at<float>(i,j))tmin=theta.at<float>(i,j);
//					if(tmax<theta.at<float>(i,j))tmax=theta.at<float>(i,j);
//				}
//			}
//		}
//	}
//
//	// Histogram of angles
//	// Amount of defined directions
//	std::vector<double> nbins;
//	std::vector<double> Hd(16);
//	double pi_part=M_PI/16;
//	for (int i=0;i<=15;i++)
//	{
//		nbins.push_back(pi_part*i);
//	}
//	for(std::size_t i=0;i<t0.size();i++)
//	{
//		double angle_val = t0[i]/pi_part;
//		double bin_num;
//		modf(angle_val, &bin_num);
//		Hd[bin_num]+=1.;
//	}
//	for(int i=0;i<16;i++)
//	{
//		Hd[i]/=t0.size();
//	}
//	// Find max in hist
//	double fmx=0;
//	double max_value=Hd[0];
//	for(size_t i=1;i<Hd.size();i++)
//	{
//		if(max_value<Hd[i])
//		{
//			max_value = Hd[i];
//			fmx=i;
//		}
//	}
//	// Check if Hd has values <> NaN
//	std::vector<double> Hd_sort(Hd.size());
//	std::vector<double> ff;
//	double lin = 0.;
//	// Shift max into center
//	if(fmx>=0)
//	{
//		if(fmx<7)
//		{
//			double shift = 7-fmx;
//			for(int i=0;i<Hd.size()-shift;i++)
//			{
//				Hd_sort[i+shift]=Hd[i];
//			}
//			for(int i=0;i<shift;i++)
//			{
//				Hd_sort[i]=Hd[(Hd.size()-shift)+i];
//			}
//		}
//		else if (fmx>8)
//		{
//			double shift = fmx-7;
//			for(int i=0;i<Hd.size()-shift;i++)
//			{
//				Hd_sort[i]=Hd[(shift)+i];
//			}
//			for(int i=0;i<=shift;i++)
//			{
//				Hd_sort[(Hd.size()-shift)+i]=Hd[i];
//			}
//		}
//		else
//		{
//			Hd_sort=Hd;
//		}
//		fmx=0;
//		double max_value=Hd_sort[0];
//		for(size_t i=1;i<Hd_sort.size();i++)
//		{
//			if(max_value<Hd_sort[i])
//			{
//				max_value = Hd_sort[i];
//				fmx=(unsigned int)i+1;
//			}
//		}
//
//		for(unsigned int i=0;i<Hd_sort.size();i++)
//		{
//			if (i+1-fmx <0)
//			{
//				ff.push_back((i+1-fmx)*(-1));
//			}
//			else
//			{
//				ff.push_back(i+1-fmx);
//			}
//		}
//
//		if(max_value>0.3)
//			max_value=0.3;
//		double sum_Hd_ff=0;
//		for (unsigned int i=3;i<Hd_sort.size()-4;i++)
//		{
//			sum_Hd_ff = sum_Hd_ff + Hd_sort[i]*ff[i];
//		}
//		lined_raw = max_value/sum_Hd_ff;
//		lin = (max_value/sum_Hd_ff-0.06)*100;
//		if(lin<1) lin=1;
//	}
//	else
//	{
//		lin = 1;
//		lined_raw = 1;
//	}
//	// rescale
//	if(lin>5) lin=5;
//	if(lin<1) lin=1;
//	results.lined=lin;


	// Checked -- Value 17
	double checked_raw1 = 0., checked_raw2 = 0.;
	// clear max. peak from histogram
	histogram.at<float>(max_p1.y) = 0.f;
	histogram.at<float>((max_p1.y+1)%histogram.rows) = 0.f;
	if (max_p1.y > 0)
		histogram.at<float>(max_p1.y-1) = 0.f;
	else
		histogram.at<float>(histogram.rows-1) = 0.f;

	// find second peak
	double max_val2=0.;
	cv::Point max_p2;
	cv::minMaxLoc(histogram, &min_val, &max_val2, &min_p, &max_p2);
	if (max_p2.y > 0)
		max_val2 += histogram.at<float>(max_p2.y-1);
	else
		max_val2 += histogram.at<float>(histogram.rows-1);
	max_val2 += histogram.at<float>((max_p2.y+1)%histogram.rows);

	// compute score
	double angular_offset = std::min(abs(max_p1.y-max_p2.y), std::min(max_p1.y, max_p2.y)+histogram_width - std::max(max_p1.y, max_p2.y)) * ((ranges_0[1]-ranges_0[0])/(double)histogram_width);
	double checked1 = 6.67*std::max(0., std::min(max_val1-0.15, 0.15));
	double checked2 = 6.67*std::max(0., std::min(max_val2-0.15, 0.15));
	double checked3 = 1. - abs(angular_offset - M_PI/2)*2/M_PI;
	checked_raw1 = max_val2;
	checked_raw2 = checked3;
	results.checked = std::max(1., std::min(5., 1. + 4 * checked1*checked2*checked3));

	if (results.checked > results.lined)
		results.lined = results.checked;

#ifdef DEBUG_OUTPUTS
	static MinMaxChecker checked_mm1;
	checked_mm1.check(checked1);
	std::cout << "checked1: " << checked1 << "  (" << checked_mm1.min << "/" << checked_mm1.max << ")" << std::endl;
	static MinMaxChecker checked_mm2;
	checked_mm2.check(checked2);
	std::cout << "checked2: " << checked2 << "  (" << checked_mm2.min << "/" << checked_mm2.max << ")" << std::endl;
	static MinMaxChecker checked_mm3;
	checked_mm3.check(checked3);
	std::cout << "checked3: " << checked3 << "  (" << checked_mm3.min << "/" << checked_mm3.max << ")" << std::endl;
#endif


//	std::vector<double> Hd_sort_checked;
//	// Switch first and second half of hist
//	for (unsigned int i=8;i<Hd_sort.size();i++)
//	{
//		Hd_sort_checked.push_back(Hd_sort[i]);
//	}
//	for(int i=0;i<8;i++)
//	{
//		Hd_sort_checked.push_back(Hd_sort[i]);
//	}
//
//	// peaks in the middle
//	std::vector< std::vector<double> > peaks;
//	double Hd_sum=0;
//	for(unsigned int i=3; i<Hd_sort_checked.size()-2;i++)
//	{
//		if(Hd_sort_checked[i]>Hd_sort_checked[i-1] && Hd_sort_checked[i]>Hd_sort_checked[i+1] && Hd_sort_checked[i]>0.05)
//		{
//			peaks.resize(peaks.size()+1);
//			peaks[peaks.size()-1].push_back(i);
//			peaks[peaks.size()-1].push_back(Hd_sort_checked[i]);
//		}
//		Hd_sum = Hd_sum + Hd_sort_checked[i]*ff[i];
//	}
//	double checked;
//	// compute value of checked
//	if(peaks.size()==0 || lin<2)
//	{
//		checked = 1;
//		checked_raw = 1;
//	}
//	else
//	{
//		cv::Mat check_x, check_y;
//		for(int i=0;i<grad_x.rows;i++)
//		{
//			for(int j=0;j<grad_x.cols;j++)
//			{
//				double val_grad_x = grad_x.at<float>(i,j);
//				double val_grad_y = grad_y.at<float>(i,j);
//				grad_x.at<float>(i,j)=round(val_grad_x*255);
//				grad_y.at<float>(i,j)=round(val_grad_y*255);
//			}
//		}
//
////		cv::Canny( grad_y, check_y,100, 300, 3);
////		cv::Canny( grad_x, check_x,100, 300, 3);
////		std::vector < std::vector<cv::Point> > lines_x;
////		std::vector < std::vector<cv::Point> > lines_y;
////		findContours(check_y, lines_y, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
////		findContours(check_x, lines_x, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
////		std::cout<<lines_x.size()<<"lines_x "<<lines_y.size()<<"lines_y "<<std::endl;
//		double contours_factor = 1;
//		if (circle_contours.size()<=10)
//			contours_factor = circle_contours.size()/20;
//		else if (circle_contours.size()>=60)
//			contours_factor = 80.*80./((double)circle_contours.size()*circle_contours.size());
//		double min_pks=peaks[0][0];
//		if(min_pks>0.3)
//			min_pks=0.3;
//		checked = 5-Hd_sum*(0.4-min_pks);
//		checked_raw = checked;
//		if(checked<1)
//			checked = 1;
//		checked = ((checked-4.7)*21+1)*contours_factor;
//		if(checked<1)checked=1;
//		if(checked>5)checked=5;
//	}
//	results.checked=checked;


	// Directionality -- Value 15
	// check if placement of few coarse primitives is on circle
	double directionality_raw1=0., directionality_raw2=0., directionality_raw3=0.;
	crit1 = 0.;
	cv::Mat large_edges_copy = large_edges.clone();
	std::vector< std::vector<cv::Point> > large_contours;
	cv::findContours(large_edges_copy, large_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point());
	std::vector< std::vector<double> > centroids(large_contours.size());
	for (size_t i=0; i<large_contours.size(); ++i)
	{
		double size = large_contours[i].size();
		double m10 = 0.;
		double m01 = 0.;
		for (size_t k=0; k<large_contours[i].size(); ++k)
		{
			m10 += large_contours[i][k].x;
			m01 += large_contours[i][k].y;
		}
		centroids[i].push_back(m10/size);
		centroids[i].push_back(m01/size);
	}
	if(centroids.size()<50 && centroids.size()>=5)
	{
		// create matrices of equation
		std::vector< std::vector<double> > circfit_a(centroids.size());
		cv::Mat amat=cvCreateMat(centroids.size(), 3, CV_32FC1);
		cv::Mat bmat= cvCreateMat(centroids.size(), 1, CV_32FC1);
		cv::Mat xmat;
		for (unsigned int i=0;i<centroids.size();i++)
		{
			float swap_var1 = -(centroids[i][0]*centroids[i][0]+centroids[i][1]*centroids[i][1]);
			float swap_var2 = centroids[i][0];
			float swap_var3 = centroids[i][1];
			if(swap_var1!=swap_var1)swap_var1=0;
			if(swap_var2!=swap_var2)swap_var2=0;
			if(swap_var3!=swap_var3)swap_var3=0;
			bmat.at<float>(i,0)= swap_var1;
			amat.at<float>(i,0)= swap_var2;
			amat.at<float>(i,1)= swap_var3;
			amat.at<float>(i,2)= 1;
		}
		std::cout << std::endl;
		// solve equation
		cv::solve(amat, bmat, xmat, cv::DECOMP_SVD);
		float a1 = xmat.at<float>(0,0);
		float a2 = xmat.at<float>(0,1);
		float a3 = xmat.at<float>(0,2);
		double xc = -.5*a1;
		double yc = -.5*a2;
		double R  =  sqrt((a1*a1+a2*a2)/4-a3);

#ifdef DEBUG_OUTPUTS
		std::cout << xc << ", " << yc << ", " << R << std::endl;
		cv::circle(large_edges, cv::Point(xc,yc), R, CV_RGB(128,128,128), 2);
		cv::imshow("large edges", large_edges);
#endif

//		std::vector<double> th;
//		double pi_part=M_PI/50;
//		for(int i=0;i<=100;i++)
//		{
//			th.push_back(pi_part*i);
//		}
//		std::vector<double>xunit;
//		std::vector<double>yunit;
//		for (unsigned int i=0; i<th.size();i++)
//		{
//			xunit.push_back(R*cos(th[i])+xc);
//			yunit.push_back(R*sin(th[i])+yc);
//		}
//		double err = 0;
//		for (unsigned int i=0;i<centroid.size();i++)
//		{
//			float dist = sqrt(edge_pixels.rows*edge_pixels.cols);
//			for(unsigned int j=0;j<xunit.size();j++)
//			{
//				double dist_check = sqrt((double)(centroid[i][0]-xunit[j])*(centroid[i][0]-xunit[j])+(centroid[i][1]-yunit[j])*(centroid[i][1]-yunit[j]));
//				if(dist>dist_check)dist=dist_check;
//			}
//			err = err+dist;
//		}
//		err = err/centroid.size();

		double err = 0.;
		for (unsigned int i=0;i<centroids.size();i++)
			err += abs(R - sqrt((double)(centroids[i][0]-xc)*(centroids[i][0]-xc)+(centroids[i][1]-yc)*(centroids[i][1]-yc))) / R;
		directionality_raw1 = err;
		crit1 = std::max(1., 5. - 10.*std::max(0., err-0.1));
	}
	else
	{
		crit1 = 1;
		directionality_raw1 = 1;
	}

	// check distribution of centroids over image
	int number_bins = 10;
	double bin_size_xhori=edge_pixels.cols/(double)number_bins;
	double bin_size_yvert=edge_pixels.rows/(double)number_bins;
	std::vector<double> hist_xhori(number_bins);
	std::vector<double> hist_yvert(number_bins);
	// compute hist_xhori and hist_yvert
	if(bin_size_xhori!=0 && bin_size_yvert!=0)
	{
		for (size_t i=0;i<centroids.size(); ++i)
		{
			int xhist_val = (int)(centroids[i][0]/bin_size_xhori);
			int yhist_val = (int)(centroids[i][1]/bin_size_yvert);
			if(xhist_val<(int)hist_xhori.size())
				hist_xhori[xhist_val]++;
			if(yhist_val<(int)hist_yvert.size())
				hist_yvert[yhist_val]++;
		}
	}
	for(size_t i=0;i<hist_xhori.size(); ++i)
	{
		hist_xhori[i]/=centroids.size();
		hist_yvert[i]/=centroids.size();
	}

//	// delete first and last position of hist
//	hist_xhori.pop_back();
//	hist_yvert.pop_back();
//	hist_xhori[0]=hist_xhori[hist_xhori.size()-1];
//	hist_yvert[0]=hist_yvert[hist_yvert.size()-1];
//	hist_xhori.pop_back();
//	hist_yvert.pop_back();
	// compute crit2 value
	//std::cout<<"point"<<std::endl;
	if (centroids.size() > 0)
	{
		cv::meanStdDev(hist_xhori, mean, stddev);
		crit2 = stddev.val[0];
		cv::meanStdDev(hist_yvert, mean, stddev);
		directionality_raw2 = crit2 + stddev.val[0];
	}
	else
		directionality_raw2 = 1.;
	crit2 = 5. - 10.*(directionality_raw2);
	crit2 = std::max(1., std::min(5., crit2));

	// check orientation of contours
//	std::cout<<"pointeccsize"<<ellipse_ecc.size()<<std::endl;
//	std::vector<double> angel(ellipse_ecc.size());
	std::vector<double> hist_angle(13);
//	std::cout<<"pointangels"<<std::endl;
	int hist_point_num=0;
//	std::cout<<"point2"<<std::endl;
	// compute hist_angle
//	std::cout<<"eccentricity size"<<eccentricity.size()<<"  "<<eccentricity[2]<<std::endl;
//	std::cout<<"ellipse_ecc[i]"<<ellipse_ecc.size()<<"  "<<ellipse_ecc[2].angle<<std::endl;
//	std::cout<<"point3"<<std::endl;

	for (unsigned int i=0;i<ellipse_ecc.size(); i++)
	{
		if(eccentricity[i]>0.7)
		{
			if(ellipse_ecc[i].angle>90)
				ellipse_ecc[i].angle=ellipse_ecc[i].angle-180;
			if(ellipse_ecc[i].angle>=0)
			{
				double angle_val = ellipse_ecc[i].angle/15;
				double bin_num;
				modf(angle_val, &bin_num);
				if((6+bin_num)<13)
					hist_angle[6+(int)bin_num]++;
				else
					std::cout<<"memory failure"<<std::endl;
			}
			else
			{
				double angle_val = -(ellipse_ecc[i].angle/15);
				double bin_num;
				modf(angle_val, &bin_num);
				if(bin_num<13)
					hist_angle[(int)bin_num]++;
				else
					std::cout<<"memory failure2"<<std::endl;
			}
			hist_point_num++;
		}
	}

	// normalize hist_angle
	double crit3 = 1.;
	if(hist_point_num>0)
	{
		for (unsigned int i=0; i<hist_angle.size(); ++i)
			hist_angle[i]/=hist_point_num;
		// compute crit3 value
		cv::meanStdDev(hist_angle, mean, stddev);
		directionality_raw3 = stddev.val[0]*stddev.val[0];
		crit3 = 50*directionality_raw3;
		crit3 = std::max(1., std::min(5., crit3));
	}

	double val15 = results.lined;
	if(val15<results.checked) val15=results.checked;
	if(val15<crit1) val15=crit1;
	if(val15<crit2) val15=crit2;
	if(val15<crit3) val15=crit3;
//	if(crit1+crit2+crit3+results.lined+results.checked <9)
//		val15=1;
	results.direct_reg=val15;
	//std::cout << crit1 << "\t" << crit2 << "\t" << crit3 << std::endl;

#ifdef DEBUG_OUTPUTS
	static MinMaxChecker directionality_mm1;
	directionality_mm1.check(directionality_raw1);
	std::cout << "directionality1: " << directionality_raw1 << "  (" << directionality_mm1.min << "/" << directionality_mm1.max << ")" << std::endl;
	static MinMaxChecker directionality_mm2;
	directionality_mm2.check(directionality_raw2);
	std::cout << "directionality2: " << directionality_raw2 << "  (" << directionality_mm2.min << "/" << directionality_mm2.max << ")" << std::endl;
	static MinMaxChecker directionality_mm3;
	directionality_mm3.check(directionality_raw3);
	std::cout << "directionality3: " << directionality_raw3 << "  (" << directionality_mm3.min << "/" << directionality_mm3.max << ")" << std::endl;
#endif


	if (raw_features != 0)
	{
		raw_features->at<float>(0, 7) = avg_primitive_size_raw1;
		raw_features->at<float>(0, 8) = avg_primitive_size_raw2;
		raw_features->at<float>(0, 9) = number_primitives_raw1;
		raw_features->at<float>(0, 10) = number_primitives_raw2;
		raw_features->at<float>(0, 11) = primitive_strength_raw;
		raw_features->at<float>(0, 12) = primitive_regularity_raw1;
		raw_features->at<float>(0, 13) = primitive_regularity_raw2;
		raw_features->at<float>(0, 14) = primitive_regularity_raw3;
		raw_features->at<float>(0, 15) = contrast_raw;
		raw_features->at<float>(0, 16) = line_likeness_raw;
		raw_features->at<float>(0, 17) = directionality_raw1;
		raw_features->at<float>(0, 18) = directionality_raw2;
		raw_features->at<float>(0, 19) = directionality_raw3;
		raw_features->at<float>(0, 20) = lined_raw;
		raw_features->at<float>(0, 21) = checked_raw1;
		raw_features->at<float>(0, 22) = checked_raw2;
	}

#ifdef DEBUG_OUTPUTS
	cv::waitKey();
#endif
}
