#include "cob_texture_categorization/color_parameter.h"


color_parameter::color_parameter()
{
}

void color_parameter::get_color_parameter_new(cv::Mat img, struct feature_results *results, cv::Mat* raw_features)
{
	// computes color parameters for the given image
	// 1. colorfulness
	//    1 = one major color, 2 = two main colors, 3 - 5 = 3-5 colors
	// 2. dominant color / 3. secondary dominant color
	//    0 = no dominant color (e.g. too colorful or transparent)
	//    1 = brown (ocher): h in [20, 70], s > 0.15, 0.2 < v < 0.7
	//    2 = red: h in [0, 20] & [345, 360], s > 0.2, v > 0.3
	//    3 = orange: h in [20, 50], s > 0.2, v > 0.3
	//    4 = yellow: h in [50, 70], s > 0.2, v > 0.3
	//    5 = green: h in [70, 160], s > 0.2, v > 0.3
	//    6 = cyan (turquoise): h in [160, 180], s > 0.2, v > 0.3
	//    7 = blue: h in [180, 250], s > 0.2, v > 0.2
	//    8 = purple: h in [250, 295], s > 0.2, v > 0.3
	//    9 = pink (rose): h in [295, 345], s > 0.2, v > 0.3
	//   10 = white: s < 0.2, v >= 0.75
	//   11 = gray (silver/metallic): s < 0.2, 0.2 < v < 0.75 or s>0.2, 0.2<v<0.3
	//   12 = black: v <= 0.2
	// 4. value/brightness / 5. variety of value/brightness
	// 6. saturation/color strength / 7. variety of saturation/color strength

	// threshold for saturation to decide whether color (hue) of pixel is defined (relevant/visible)
	double saturation_color_threshold=0.25;

	// transform into HSV color space  (h [0, 360], s,v [0, 1])
	cv::Mat hsv;
	cv::cvtColor(img, hsv, CV_BGR2HSV);

	// COLOR
	double colorfulness=0.;
	double colorfulness_raw=0.;
	double dom_color;
	double dom_color2;
	std::vector<double> color_histogram;
//	for(int i=0;i<hsv.rows;i++)
//	{
//		for(int j=0;j<hsv.cols;j++)
//		{
//			const cv::Vec3b& value = hsv.at<cv::Vec3b>(i,j);
//			if(comp_val>threshold*255)
//			{
//				double h_val = (hsv.at<cv::Vec3b>(i,j)[0]);
//				rel_pix.push_back(h_val/180);
//			}
//		}
//	}
	// SATURATION AND VALUE
	// calculate mean and standard deviation of saturation and value for all relevant pixels
	cv::Scalar means, stds, meanv, stdv;
	double s_mean, s_std, v_mean, v_std;
	double s_mean_raw, s_std_raw, v_mean_raw, v_std_raw;
	std::vector<double> s, v;
	for(int i=0;i<hsv.rows;i++)
	{
		for(int j=0;j<hsv.cols;j++)
		{
			double val_s = hsv.at<cv::Vec3b>(i,j)[1];
			double val_v = hsv.at<cv::Vec3b>(i,j)[2];
			s.push_back(val_s/255.);
			v.push_back(val_v/255.);
		}
	}
	cv::meanStdDev(s, means, stds);
	cv::meanStdDev(v, meanv, stdv);
	s_mean = means.val[0]*6+0.5;
	s_mean_raw = means.val[0];
	s_std = stds.val[0]*10+0.5;
	s_std_raw = stds.val[0];
	v_mean = meanv.val[0]*6+0.5;
	v_mean_raw = meanv.val[0];
	v_std = stdv.val[0]*12+0.5;
	v_std_raw = stdv.val[0];

	s_mean = std::max(1., std::min(5., s_mean));
	s_std = std::max(1., std::min(5., s_std));
	v_mean = std::max(1., std::min(5., v_mean));
	v_std = std::max(1., std::min(5., v_std));
//	//no color in Image
//	if(s_mean < 1.2 && s_std <1.2)
//	{
//		colorfulness = 0;
//		dom_color 	 = 0;
//		dom_color2	 = 0;
//	}

	// Submit results
	(*results).colorfulness = colorfulness;
	(*results).dom_color = dom_color;
	(*results).dom_color2 = dom_color2;
	(*results).s_mean = s_mean;
	(*results).s_std = s_std;
	(*results).v_mean = v_mean;
	(*results).v_std = v_std;

	// raw values
	if (raw_features != 0)
	{
		raw_features->at<float>(0, 0) = colorfulness_raw;
		raw_features->at<float>(0, 1) = (*results).dom_color;
		raw_features->at<float>(0, 2) = (*results).dom_color2;
		raw_features->at<float>(0, 3) = v_mean_raw;
		raw_features->at<float>(0, 4) = v_std_raw;
		raw_features->at<float>(0, 5) = s_mean_raw;
		raw_features->at<float>(0, 6) = s_std_raw;
	}
}


void color_parameter::get_color_parameter(cv::Mat img, struct feature_results *results, cv::Mat* raw_features)
{
//	computes color parameters for all images of a given folder
//
//	INPUT
//	ROI of source image
//	struct results stores all 17 texture features
//
//	OUTPUT
//	update of results struct with color features:
//
//	Value 1: mean of saturation - scale 1 to 5
//	Value 2: standard deviation of saturation - scale 1 to 5
//	Value 3: mean of value - scale 1 to 5
//	Value 4: standard deviation of value - scale 1 to 5
//	Value 5: colorfulness - scale 1 to 5: 1=plain-colored, 2=bi-colored, 3-5=multi-colored
//	Value 6: dominant color
//	Value 7: secondary dominant color
//
//	color details
//	std::vector<double> centers;
//	for(int i=0;i<21;i++)
//	{
//		centers.push_back(i*0.05);
//	}

//	threshold for saturation to decide whether pixel is on object or part of
//	white background: relevant if input "white_back" is true (images show
//	an object in front of white background)
//	double threshold_white=0.25;

//	threshold for saturation to decide whether color (hue) of pixel is
//	relevant/visible: should be larger or equal than "threshold_white" if
//	input "white_back" is true (images show an object in front of white
//	background)
	double threshold=0.25;

//	ratio of relevant pixels (saturation > threshold) to detect colors (hue)
	double ratio=0.1;

//	min hue value in normalized histogram to detect peaks and value to accept
//	color as represented in the image (for determination of colorfulness)
//	double threshold_color=0.03;

//	ratio of colors that have to be represented in the image that can be
//	characterized as maximum colorful (colorfulness = 5)
//	double colorful5=0.38;
//	colorful5=0.28;

//	transform into HSV color space
	cv::Mat hsv;
	cv::cvtColor(img, hsv, CV_BGR2HSV);

//	int testcount = 0;

	double colorfulness=0.;
	double colorfulness_raw=0.;
	double dom_color;
	double dom_color2;

//  HUE
//  look at relevant pixels
	std::vector<double> rel_pix;
	for(int i=0;i<hsv.rows;i++)
	{
		for(int j=0;j<hsv.cols;j++)
		{
			double comp_val = hsv.at<cv::Vec3b>(i,j)[1];
			if(comp_val>threshold*255)
			{
				double h_val = (hsv.at<cv::Vec3b>(i,j)[0]);
				rel_pix.push_back(h_val/180);
//				testcount++;
			}
		}
	}
//	std::cout<<testcount<<"testcournt;"<<std::endl;
//  check if ratio of image pixels having a visible color is
//  sufficient (>ratio)
	std::vector<double> hue_hist(21);
//	std::vector<std::vector <double> > peaks;
//	double pb=0;
//	double b_peak_pos=-1;
	if(rel_pix.size() > hsv.rows*hsv.cols * ratio)
	{
		for(unsigned int i=0;i<rel_pix.size();i++)
		{
			double hist_val = (rel_pix[i]*40.0);	// rel_pix[i]/0.025);
			double bin_num;
			modf(hist_val, &bin_num);
			bin_num= ceil(bin_num/2);
			if(bin_num<hue_hist.size())
			{
				hue_hist[bin_num]++;
			}
		}
		for(unsigned int i=0;i<hue_hist.size();i++)
		{
			hue_hist[i]/=(double)rel_pix.size();
		}
////		peaks in the middle
//		for(unsigned int i=1; i<hue_hist.size()-1;i++)
//		{
//			if(hue_hist[i]>hue_hist[i-1] && hue_hist[i]>hue_hist[i+1] && hue_hist[i]>threshold_color)
//			{
//				peaks.resize(peaks.size()+1);
//				peaks[peaks.size()-1].push_back(i);
//				peaks[peaks.size()-1].push_back(hue_hist[i]);
//			}
//		}
////		boundary value
////		check left bound
//		double lb_peak=0;
//		double rb_peak=0;
//		if(hue_hist[0]-hue_hist[hue_hist.size()-1]>threshold_color && hue_hist[0]-hue_hist[1]>threshold_color)
//		{
//			lb_peak = hue_hist[0];
//		}
////		check right bound
//		if(hue_hist[hue_hist.size()-1]-hue_hist[hue_hist.size()-2]>threshold_color && hue_hist[hue_hist.size()-1]-hue_hist[0]>threshold_color)
//		{
//			rb_peak = hue_hist[hue_hist.size()-1];
//		}
//		if(lb_peak>0 && rb_peak<=0)
//		{
////			peak on left
//			pb=1;
//			b_peak_pos = hue_hist.size()-1;
//		}else if(lb_peak<=0 && rb_peak>0)
//		{
////			peak on right
//			pb=1;
//			b_peak_pos = hue_hist.size()-1;
//		}else if(lb_peak>0 && rb_peak>0)
//		{
////			one peak --->take max
//			pb=1;
//			if(hue_hist[0]>hue_hist[hue_hist.size()-1])
//			{
//				b_peak_pos=0;
//			}else
//			{
//				b_peak_pos=hue_hist.size()-1;
//			}
//		}else
//		{
//			pb=0; //no peak
//			b_peak_pos=-1;
//		}
	}
	else
	{
		colorfulness = 0;
		colorfulness_raw = 0;
		dom_color = 0;
		dom_color2 = 0;
	}

	std::cout << "\nhue histogram:  ";
	for(size_t i=0;i<hue_hist.size();i++)
		std::cout<<hue_hist[i]<<"  ";
	std::cout << std::endl;

	//colorfulness
	int amount_of_color=0;

	//dom_color & dom_color2
	cv::Mat hue(1,hue_hist.size(),CV_32F);
	for(size_t i=0;i<hue_hist.size();i++)
	{
		hue.at<float>(0,i)=hue_hist[i];
//		if(hue_hist[i]>threshold-0.1)amount_of_color++;
	}
	cv::Point min_loc, max_loc, max_loc2;
	double min1, max1;
	cv::minMaxLoc(hue, &min1, &max1, &min_loc, &max_loc);
	hue.at<float>(max_loc)=0;
	cv::minMaxLoc(hue, &min1, &max1, &min_loc, &max_loc2);

	for(size_t i=0;i<hue_hist.size();i++)
	{
		if((hue_hist[max_loc.x])*0.25 < hue_hist[i]) amount_of_color++;
	}

//	std::cout<<std::endl<<max_loc<<"  "<<max_loc2<<std::endl;
//	std::cout<<max_loc.x<<"--"<<(double)max_loc.x/2<<"  "<<max_loc2.x<<"--"<<(double)max_loc2.x/2<<"  ";
	dom_color = ((double)max_loc.x+1.)/2.;
	dom_color2 = ((double)max_loc2.x+1.)/2.;
	if(dom_color<1)dom_color=1;
	if(dom_color>10)dom_color=10;
	if(dom_color2<1)dom_color2=1;
	if(dom_color2>10)dom_color2=10;
	if(dom_color==1 && dom_color2==1) dom_color2=1.5;
	std::cout<< "dominant colors: " << dom_color<<", "<<dom_color2<<std::endl;


	colorfulness = amount_of_color;
	if(colorfulness <1) colorfulness=1;
	if(colorfulness>5) colorfulness=5;

	if(colorfulness==1) dom_color2=0;

	// colorfulness
	// plain-colored --> dominant color
//	double swap=0;
//	double swap1=0;
//	if(peaks.size()+pb==1)
//	{
//		colorfulness=1;
//		colorfulness_raw=1;
//		if(peaks.size()>0)
//		{
//			swap = (peaks[0][0]-1)/2;
//			dom_color = swap;
//		}else{
//			swap = (b_peak_pos-1)/2;
//			dom_color = swap;
//		}
//		dom_color2=0;
//	}else if(peaks.size()+pb==2)
//	{
//		// bi-colored --> dominant and secondary dominant color
//		colorfulness=2;
//		colorfulness_raw=2;
//		if(peaks.size()==2)
//		{
//			if(peaks[0][0]>peaks[1][0])
//			{
//				swap = (peaks[0][0]-1)/2;
//				swap1 = (peaks[1][0]-1)/2;
//				dom_color = swap;
//				dom_color2= swap1;
//			}else{
//				swap = (peaks[1][0]-1)/2;
//				swap1 = (peaks[0][0]-1)/2;
//				dom_color = swap;
//				dom_color2= swap1;
//			}
//		}
//	}
//	else
//	{
//		double num_non_zeros=0;
//		for(unsigned int i=0;i<hue_hist.size();i++)
//		{
//			if(hue_hist[i]>threshold_color)num_non_zeros++;
//		}
//		if(hue_hist.size()>0)
//		{
//			colorfulness = ((num_non_zeros/hue_hist.size())*5)/colorful5;
//			colorfulness_raw = ((num_non_zeros/hue_hist.size())*5);
//		}
//		if(colorfulness>5)
//		{
//			colorfulness = 5;
//		}
//
//		if(peaks.size()>2)
//		{
//			if(peaks[0][1]>peaks[1][1])
//			{
//				swap = (peaks[0][0])/1;
//				swap1 = (peaks[1][0])/1;
//				dom_color = swap;
//				dom_color2 = swap1;
//			}else{
//				swap = (peaks[1][0])/1;
//				swap1 = (peaks[0][0])/1;
//				dom_color = swap;
//				dom_color2 = swap1;
//			}
//		}
//		for(unsigned int i=2;i<peaks.size();i++)
//		{
//			if(dom_color>=0 && dom_color<peaks.size() && dom_color2>=0 && dom_color2<peaks.size())
//			{
//				if(round(dom_color2)<peaks.size() && round(dom_color2)>=0)
//				{
//					if(peaks[i][1]>peaks[round(dom_color2)][1])
//					{
//						if(peaks[i][1]>peaks[round(dom_color)][1])
//						{
//							dom_color2=dom_color;
//							dom_color = i;
//						}else{
//							dom_color2=i;
//						}
//					}
//				}
//			}
//		}
//		if(b_peak_pos>=0 && b_peak_pos<peaks.size())
//		{
//			if(dom_color>=0&&dom_color<peaks.size())
//			{
//				if(peaks[round(b_peak_pos)][0]>peaks[round(dom_color)][0])
//				{
//					dom_color2=dom_color;
//					dom_color = b_peak_pos;
//				}else{
//					dom_color2=b_peak_pos;
//				}
//			}
//		}
//	}

//	rescale for yellow and green
	if(dom_color>=1.5 && dom_color<2)
		dom_color=dom_color+0.5;
	else if(dom_color>=2 && dom_color<=2.5)
		dom_color = dom_color+1;
	if(dom_color2>=1.5 && dom_color<2)
		dom_color2=dom_color2 + 0.5;
	else if(dom_color2>=2 && dom_color2<=2.5)
		dom_color2 = dom_color2 + 1;

//	if(dom_color<1) dom_color=1;
//	if(dom_color>10)dom_color=10;
//	if(dom_color2<1) dom_color2=1;
//	if(dom_color2>10)dom_color2=10;

	if(dom_color!=dom_color)dom_color=0;
	if(dom_color2!=dom_color2)dom_color2=0;

//  SATURATION AND VALUE
//  calculate mean and standard deviation of saturation and value for
//  all relevant pixels

//  adopted measurement
	cv::Scalar means, stds, meanv, stdv;
	double s_mean, s_std, v_mean, v_std;
	double s_mean_raw, s_std_raw, v_mean_raw, v_std_raw;
	std::vector<double> s, v;
	for(int i=0;i<hsv.rows;i++)
	{
		for(int j=0;j<hsv.cols;j++)
		{
			double val_s = hsv.at<cv::Vec3b>(i,j)[1];
			double val_v = hsv.at<cv::Vec3b>(i,j)[2];
			s.push_back(val_s/255);
			v.push_back(val_v/255);
		}
	}
	cv::meanStdDev(s, means, stds);
	cv::meanStdDev(v, meanv, stdv);
	s_mean = means.val[0]*6+0.5;
	s_mean_raw = means.val[0];
	s_std = stds.val[0]*10+0.5;
	s_std_raw = stds.val[0];
	v_mean = meanv.val[0]*6+0.5;
	v_mean_raw = meanv.val[0];
	v_std = stdv.val[0]*12+0.5;
	v_std_raw = stdv.val[0];

	if(s_mean>1)
	{
		if(s_mean>5)s_mean=5;
	}else{
		s_mean=1;
	}
	if(s_std>1)
	{
		if(s_std>5)s_std=5;
	}else{
		s_std=1;
	}
	if(v_mean>1)
	{
		if(v_mean>5)v_mean=5;
	}else{
		v_mean=1;
	}
	if(v_std>1)
	{
		if(v_std>5)v_std=5;
	}else{
		v_std=1;
	}
	//no color in Image
	if(s_mean < 1.2 && s_std <1.2)
	{
		colorfulness = 0;
		dom_color 	 = 0;
		dom_color2	 = 0;
	}

//	Submit results
	(*results).s_mean = s_mean;
	(*results).s_std = s_std;
	(*results).v_mean = v_mean;
	(*results).v_std = v_std;
	(*results).colorfulness = colorfulness;
	(*results).dom_color = dom_color;
	(*results).dom_color2 = dom_color2;
//		(*results).dom_color = 1;
//		(*results).dom_color2 = 2;

	// raw values
	if (raw_features != 0)
	{
		raw_features->at<float>(0, 0) = colorfulness_raw;
		raw_features->at<float>(0, 1) = (*results).dom_color;
		raw_features->at<float>(0, 2) = (*results).dom_color2;
		raw_features->at<float>(0, 3) = v_mean_raw;
		raw_features->at<float>(0, 4) = v_std_raw;
		raw_features->at<float>(0, 5) = s_mean_raw;
		raw_features->at<float>(0, 6) = s_std_raw;
	}
}
