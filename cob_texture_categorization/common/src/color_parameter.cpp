#include "color_parameter.h"

color_parameter::color_parameter()
{
}

void color_parameter::get_color_parameter(cv::Mat img, struct feature_results *results)
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
	std::vector<double> centers;
	for(int i=0;i<21;i++)
	{
		centers.push_back(i*0.05);
	}

//	threshold for saturation to decide whether pixel is on object or part of
//	white background: relevant if input "white_back" is true (images show
//	an object in front of white background)
	double threshold_white=0.25;

//	threshold for saturation to decide whether color (hue) of pixel is
//	relevant/visible: should be larger or equal than "threshold_white" if
//	input "white_back" is true (images show an object in front of white
//	background)
	double threshold=0.25;

//	ratio of relevant pixels (saturation > threshold) to detect colors (hue)
	double ratio=0.1;

//	min hue value in normalized histogram to detect peaks and value to accept
//	color as represented in the image (for determination of colorfulness)
	double threshold_color=0.03;

//	ratio of colors that have to be represented in the image that can be
//	characterized as maximum colorful (colorfulness = 5)
	double colorful5=0.38;
//	colorful5=0.28;

//	transform into HSV color space
	cv::Mat hsv;
	cv::cvtColor(img, hsv, CV_BGR2HSV);

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
			}
		}
	}

//  check if ratio of image pixels having a visible color is
//  sufficient (>ratio)
	std::vector<double> hue_hist(21);
	std::vector<std::vector <double> > peaks;
	double pb=0;
	double b_peak_pos=-1;
	if((rel_pix.size()/hsv.rows*hsv.cols)>ratio)
	{
		for(int i=0;i<rel_pix.size();i++)
		{
			double hist_val = (rel_pix[i]/0.025);
			double bin_num;
			modf(hist_val, &bin_num);
			bin_num= ceil(bin_num/2);
			if(bin_num<hue_hist.size())
			{
			hue_hist[bin_num]=hue_hist[bin_num]+1;
			}
		}
		for(int i=0;i<hue_hist.size();i++)
		{
			hue_hist[i]=hue_hist[i]/rel_pix.size();
		}
//		peaks in the middle
		for(int i=1; i<hue_hist.size()-1;i++)
		{
			if(hue_hist[i]>hue_hist[i-1] && hue_hist[i]>hue_hist[i+1] && hue_hist[i]>threshold_color)
			{
				peaks.resize(peaks.size()+1);
				peaks[peaks.size()-1].push_back(i);
				peaks[peaks.size()-1].push_back(hue_hist[i]);
			}
		}
//		boundary value
//		check left bound
		double lb_peak=0;
		double rb_peak=0;
		if(hue_hist[0]-hue_hist[hue_hist.size()-1]>threshold_color && hue_hist[0]-hue_hist[1]>threshold_color)
		{
			lb_peak = hue_hist[0];
		}
//		check right bound
		if(hue_hist[hue_hist.size()-1]-hue_hist[hue_hist.size()-2]>threshold_color && hue_hist[hue_hist.size()-1]-hue_hist[0]>threshold_color)
		{
			rb_peak = hue_hist[hue_hist.size()-1];
		}
		if(lb_peak>0 && rb_peak<=0)
		{
//			peak on left
			pb=1;
			b_peak_pos = hue_hist.size()-1;
		}else if(lb_peak<=0 && rb_peak>0)
		{
//			peak on right
			pb=1;
			b_peak_pos = hue_hist.size()-1;
		}else if(lb_peak>0 && rb_peak>0)
		{
//			one peak --->take max
			pb=1;
			if(hue_hist[0]>hue_hist[hue_hist.size()-1])
			{
				b_peak_pos=0;
			}else
			{
				b_peak_pos=hue_hist.size()-1;
			}
		}else
		{
			pb=0; //no peak
			b_peak_pos=-1;
		}
	}

//	colorfullness
//	plaint-colored --> dominant color
	double colorfulness;
	double dom_color;
	double dom_color2;
	double swap=0;
	double swap1=0;
	if(peaks.size()+pb==1)
	{
		colorfulness=1;
		if(peaks.size()>0)
		{
			swap = (peaks[0][0]-1)/2;
			dom_color = swap;
		}else{
			swap = (b_peak_pos-1)/2;
			dom_color = swap;
		}
		dom_color2=0;
	}else if(peaks.size()+pb==2)
	{
//		bi-colored --> dominant and secondary dominant color
		colorfulness=2;
		if(peaks.size()==2)
		{
			if(peaks[0][0]>peaks[1][0])
			{
				swap = (peaks[0][0]-1)/2;
				swap1 = (peaks[1][0]-1)/2;
				dom_color = swap;
				dom_color2= swap1;
			}else{
				swap = (peaks[1][0]-1)/2;
				swap1 = (peaks[0][0]-1)/2;
				dom_color = swap;
				dom_color2= swap1;
			}
		}
	}else{
		double num_non_zeros;
		for(int i=0;i<hue_hist.size();i++)
		{
			if(hue_hist[i]>threshold_color)num_non_zeros++;
		}
		if(hue_hist.size()>0)
		{
		colorfulness = ((num_non_zeros/hue_hist.size())*5)/colorful5;
		}
		if(colorfulness>5)
		{
			colorfulness = 5;
		}

		if(peaks.size()>2)
		{
			if(peaks[0][1]>peaks[1][1])
			{
				swap = (peaks[0][0])/1;
				swap1 = (peaks[1][0])/1;
				dom_color = swap;
				dom_color2 = swap1;
			}else{
				swap = (peaks[1][0])/1;
				swap1 = (peaks[0][0])/1;
				dom_color = swap;
				dom_color2 = swap1;
			}
		}
		for(int i=2;i<peaks.size();i++)
		{
			if(dom_color>=0 && dom_color<peaks.size() && dom_color2>=0 && dom_color2<peaks.size())
			{
				if(round(dom_color2)<peaks.size() && round(dom_color2)>=0)
				{
					if(peaks[i][1]>peaks[round(dom_color2)][1])
					{
						if(peaks[i][1]>peaks[round(dom_color)][1])
						{
							dom_color2=dom_color;
							dom_color = i;
						}else{
							dom_color2=i;
						}
					}
				}
			}
		}
		if(b_peak_pos>=0 && b_peak_pos<peaks.size())
		{
			if(dom_color>=0&&dom_color<peaks.size())
			{
				if(peaks[round(b_peak_pos)][0]>peaks[round(dom_color)][0])
				{
					dom_color2=dom_color;
					dom_color = b_peak_pos;
				}else{
					dom_color2=b_peak_pos;
				}
			}
		}
	}

//	rescale for yellow and green
	if(dom_color==1.5){
		dom_color=dom_color+0.5;
	}else if(dom_color>=2 && dom_color<=2.5)
	{
		dom_color = dom_color+1;
	}
	if(dom_color2==1.5){
		dom_color2=dom_color2 + 0.5;
	}else if(dom_color2>=2 && dom_color2<=2.5)
	{
		dom_color2 = dom_color2 + 1;
	}
	if(dom_color!=dom_color)dom_color=0;
	if(dom_color2!=dom_color2)dom_color2=0;

//  SATURATION AND VALUE
//  calculate mean and standard deviation of saturation and value for
//  all relevant pixels

//  adopted measurement
	cv::Scalar means, stds, meanv, stdv;
	double s_mean, s_std, v_mean, v_std;
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
	s_std = stds.val[0]*10+0.5;
	v_mean = meanv.val[0]*6+0.5;
	v_std = stdv.val[0]*12+0.5;

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

}
