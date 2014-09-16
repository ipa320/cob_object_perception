#include "cob_texture_categorization/splitandmerge.h"
#include "cob_texture_categorization/create_lbp.h"
#include "cob_texture_categorization/texture_features.h"
#include "cob_texture_categorization/color_parameter.h"
#include <math.h>

#include <sys/time.h>

int mergefirstval = 2;
int mergesecondval = 150;
double splitval = 5.1;

//Struct represents regions/nodes in tree
//struct region {
//	std::vector<region> childs;
//	std::vector<int> neighbors;
//    bool validity;
//    std::vector<double> lbp;
//    bool lbp_set;
//    int class_num;
//    int id;
//    cv::Rect roi;
//
//};
std::vector<bool> validity;


struct region {
std::vector<region> childs;
std::vector<int> neighbors, N, E, S, W;
std::vector<bool> Nb, Eb, Sb, Wb;
bool validity;
std::vector<double> lbp;
bool lbp_set;
int class_num;
int id;
cv::Rect roi;
};
region r;
//Merged region class
struct region_values{
	std::vector<int> members;
	std::vector<struct feature_results> feat_res;
	int r,g,b;
	std::vector<double> values;
	bool merged;
	bool merged_second;
	int merged_pos;
	std::vector<int> merged_to;
};
cv::Mat original_image;


//Checks if two regions are neighbors
bool neighbor(region n, region o)
{
	int x11 = n.roi.x;
	int x12 = n.roi.x + n.roi.width;
	int x21 = o.roi.x;
	int x22 = o.roi.x + o.roi.width;

	int y11 = n.roi.y;
	int y12 = n.roi.y + n.roi.height;
	int y21 = o.roi.y;
	int y22 = o.roi.y + o.roi.height;


//	// Check neighbor on y-Axis
//	if( abs(x12-x21)<3 || abs(x11-x22)<3 || abs(x11-x21)<3 || abs(x12-x22)<3)
//	{
//		if(x12 < x21 || x22 < x11)
//		{
//			return false;
//		}else{
//			return true;
//		}
//
//	}
//	//Check neighbor on x-Axis
//	if(abs(y11-y21)<3 || abs(y12-y22)<3 || abs(y11-y22)<3 || abs(y12-y21)<3)
//	{
//		if(y12 < y21 || y22 < y11)
//				{
//					return false;
//				}else{
//					return true;
//				}
//	}

	// Check neighbor on y-Axis
	if( abs(x12-x21)<3 || abs(x11-x22)<3 || abs(x11-x21)<3 || abs(x12-x22)<3)
	{
		if((y11 >= y21 && y11 <= y22) || (y12 >= y22 && y12 <= y21))
		{
			return true;
		}
		if((y21 >= y11 && y21 <= y12) || (y22 >= y12 && y22 <= y11))
		{
			return true;
		}
	}
	//Check neighbor on x-Axis
	if(abs(y11-y21)<3 || abs(y12-y22)<3 || abs(y11-y22)<3 || abs(y12-y21)<3)
	{
		if((x11 >= x21 && x11 <= x22) || (x12 >= x22 && x12 <= x21))
		{
			return true;
		}
		if((x21 >= x11 && x21 <= x12) || (x22 >= x12 && x22 <= x11))
		{
			return true;
		}
	}

//	// Check neighbor on y-Axis
//	if( x12 == x21 || x11 == x22 || x11 == x21 || x12 == x22)
//	{
//		if((y11 >= y21 && y12 <= y22) || (y21 >= y11 && y22 <= y21))
//		{
//			return true;
//		}
//	}
//	//Check neighbor on x-Axis
//	if(y11 == y21 || y12 == y22 || y11==y22 || y12 == y21)
//	{
//		if((x11 >= x21 && x12 <= x22) || (x21 >= x11 && x22 <= x21))
//		{
//			return true;
//		}
//	}
	return false;
}

void get_neighbor(region n, std::vector<int> &ids)
{
	if(n.childs.size()<1)
	{
		ids.push_back(n.id);
	}else{
		std::vector<int> a;
		get_neighbor(n.childs[0], ids);
		get_neighbor(n.childs[1], ids);
		get_neighbor(n.childs[2], ids);
		get_neighbor(n.childs[3], ids);
	}
}


//compares 4 lbp hist with chisquare test
double lbp_check_chi_four(double *hist1, double *hist2,double *hist3, double *hist4, int size)
{
//	double result=0;
//	double diff;
	double kat[10];
	double prob[4];
	double sum=0;
	double sum1=0;
	double sum2=0;
	double x=0;


	for(int j=0; j<10;j++)
	{
		prob[0]=prob[0]+hist1[j];
		prob[1]=prob[1]+hist2[j];
		prob[2]=prob[2]+hist3[j];
		prob[3]=prob[3]+hist4[j];

		sum1 = sum1+ hist1[j]+hist2[j]+hist3[j]+hist4[j];
//		if(prob[0]!=prob[0]||prob[1]!=prob[1]||prob[2]!=prob[2]||prob[3]!=prob[3]|| sum1!=sum1)
//			{
//				std::cout << "firsterror"<<std::endl;
//				for(int i=0;i<10;i++)
//				{
//					std::cout << hist1[i]<<"a "<<hist2[i]<<"b "<<hist3[i]<<"c "<<hist4[i]<<"d ";
//				}std::cout<<std::endl;
//			}
	}
	for(int j=0;j<10;j++)
	{
				kat[j]=hist1[j]+hist2[j]+hist3[j]+hist4[j];
				sum2 = sum2 +hist1[j]+hist2[j]+hist3[j]+hist4[j];
				if(kat[j]!=kat[j])std::cout<<"errorwhile"<<j<<" "<<std::endl;
				if(kat[j]!=kat[j]||sum2!=sum2)std::cout << "seconderror"<<std::endl;
	}
	sum=sum1+sum2;

	for(int i = 0;i<4;i++){
		int count=0;
		for(int j=0;j<10;j++)
		{
			if(prob[i]*kat[j]!=0 && sum!=0)x=x+(pow((hist1[j]-(prob[i]*kat[j])/sum),2)/((prob[i]*kat[j])/sum));
//			if(x!=x)std::cout << "xerror "<< hist1[j]<<"hist1 "<< prob[i]<< "prob "<<kat[j] << "kat "<<sum<<"sum "<<std::endl;
//			std::cout << x<<":"<<j<<"j"<<i<<"i"<<std::endl;
			if(prob[i]*kat[j]!=0 && sum!=0)x=x+(pow((hist2[j]-(prob[i]*kat[j])/sum),2)/((prob[i]*kat[j])/sum));
//			if(x!=x)std::cout << "xerror "<< hist1[j]<<"hist1 "<< prob[i]<< "prob "<<kat[j] << "kat "<<sum<<"sum "<<std::endl;
//			std::cout << x<<":"<<j<<"j"<<i<<"i"<<std::endl;
			if(prob[i]*kat[j]!=0 && sum!=0)x=x+(pow((hist3[j]-(prob[i]*kat[j])/sum),2)/((prob[i]*kat[j])/sum));
//			if(x!=x)std::cout << "xerror "<< hist1[j]<<"hist1 "<< prob[i]<< "prob "<<kat[j] << "kat "<<sum<<"sum "<<std::endl;
//			std::cout << x<<":"<<j<<"j"<<i<<"i"<<std::endl;
			if(prob[i]*kat[j]!=0 && sum!=0) x=x+(pow((hist4[j]-(prob[i]*kat[j])/sum),2)/((prob[i]*kat[j])/sum));
//			std::cout << x<<":"<<j<<"j"<<i<<"i"<<std::endl;
//			if(x!=x)std::cout << "xerror "<< hist1[j]<<"hist1 "<< prob[i]<< "prob "<<kat[j] << "kat "<<sum<<"sum "<<std::endl;
//			if(((prob[i]*kat[j])/sum!=(prob[i]*kat[j])/sum) && count < 1)std::cout << "xerror1 "<< hist1[j]<<"hist1 "<< prob[i]<< "prob "<<kat[j] << "kat "<<sum<<"sum "<<std::endl;
//			if(pow((hist4[j]-(prob[i]*kat[j])/sum),2)!=pow((hist4[j]-(prob[i]*kat[j])/sum),2) && count < 1)std::cout << "xerror2 "<< hist1[j]<<"hist1 "<< prob[i]<< "prob "<<kat[j] << "kat "<<sum<<"sum "<<std::endl;
		}
		count++;
	}
//	std::cout <<x<<"chiquadrat"<<std::endl;
	x=x/(size);
//	std::cout <<x<<"chiquadrat/size"<<std::endl;
	return x;

}
//compares all lbp hist in input vector wiht chisquare test
double lbp_check_chi(std::vector< std::vector<double> > &hist, int point_number, bool merge)
{

//	for(int i=0;i<hist.size();i++)
//	{
//		for(int j=0;j<hist[i].size();j++)
//		{
//			std::cout<<hist[i][j]<<" ";
//		}std::cout<<"|||";
//	}
//	std::cout<<"ENDE"<<std::endl;

	int size = hist.size();
	int size_bins = hist[0].size();
//	double result=0;
//	double diff;
	std::vector<double> kat;
	std::vector<double> prob;
	kat.resize(size_bins);
	prob.resize(size);
	double sum=0;
	double sum1=0;
	double sum2=0;
	double x=0;
	if(point_number == 0)point_number=1;

	for(int i=0; i<size; i++)
	{
		for(int j=0; j<size_bins;j++)
		{
			prob[i]=prob[i]+hist[i][j];
			sum1 = sum1+ hist[i][j];
		}
	}
	for(int j=0;j<size;j++)
	{
		for(int i=0; i<size_bins; i++)
		{
//					std::cout<<kat[i]<<"kati "<<hist[j][i]<<"hist"<<std::endl;
					kat[i]=kat[i]+hist[j][i];
					sum2 = sum2 + hist[j][i];

					if(kat[i]!=kat[i] && merge)std::cout<<"errrorwhile"<<i<<" "<<std::endl;
					if((kat[i]!=kat[i]||sum2!=sum2) && merge)std::cout << "seconderror"<<std::endl;
		}
	}
	sum=sum1+sum2;
	for(int i = 0;i<size;i++){
		for(int j=0;j<size_bins;j++)
		{
			if(prob[i]*kat[j]!=0 && sum!=0) x=x+(pow((hist[i][j]-(prob[i]*kat[j])/sum),2)/((prob[i]*kat[j])/sum));
			if(x!=x && merge)std::cout << "xerror "<< hist[i][j]<<"hist1 "<< prob[i]<< "prob "<<kat[j] << "kat "<<sum<<"sum "<<std::endl;
		}
	}
	x=(4*x)/(point_number);
	return x;
}


//checks two images on hsv homogenity
bool check_hsv(cv::Mat img, cv::Mat img2)
{
	double rows = img.rows;
	double cols = img.cols;
	double rows2 = img2.rows;
	double cols2 = img2.cols;
	//get HSV value
		cv::Mat hsv;
		cv::cvtColor(img, hsv, CV_BGR2HSV);
		cv::Mat hsv2;
		cv::cvtColor(img2, hsv2, CV_BGR2HSV);
		int h = 0;
		int h2 = 0;
		int s = 0;
		int v = 0;
		int hmin = 0;
		int hmin2 = 0;
		int smin = 255;
		int vmin = 255;
		int hmax = 0;
		int hmax2 = 0;
		int smax = 0;
		int vmax = 0;

		for(int i=0;i<rows;i++)
		{
			for(int j=0;j<cols;j++)
			{
				if(hmin > hsv.at<cv::Vec3b>(i,j)[0]) hmin = hsv.at<cv::Vec3b>(i,j)[0];
				if(hmax < hsv.at<cv::Vec3b>(i,j)[0]) hmax = hsv.at<cv::Vec3b>(i,j)[0];
				if(hmin2 > hsv2.at<cv::Vec3b>(i,j)[0]) hmin2 = hsv2.at<cv::Vec3b>(i,j)[0];
				if(hmax2 < hsv2.at<cv::Vec3b>(i,j)[0]) hmax2 = hsv2.at<cv::Vec3b>(i,j)[0];
				if(smin > hsv.at<cv::Vec3b>(i,j)[1]) smin = hsv.at<cv::Vec3b>(i,j)[1];
				if(smax < hsv.at<cv::Vec3b>(i,j)[1]) smax = hsv.at<cv::Vec3b>(i,j)[1];
				if(vmin > hsv.at<cv::Vec3b>(i,j)[2]) vmin = hsv.at<cv::Vec3b>(i,j)[2];
				if(vmax < hsv.at<cv::Vec3b>(i,j)[2]) vmax = hsv.at<cv::Vec3b>(i,j)[2];
				h = h + hsv.at<cv::Vec3b>(i,j)[0];
				h2 = h2 + hsv2.at<cv::Vec3b>(i,j)[0];
				s = s + hsv.at<cv::Vec3b>(i,j)[1];
				v = v + hsv.at<cv::Vec3b>(i,j)[2];
			}
		}
		h = h/(rows*cols);
		h2 = h2/(rows2*cols2);
		s = s/(rows*cols);
		v = v/(rows*cols);

		if((hmax<hmax2*0.4 || hmax>hmax2*1.6)) return false;
		if((hmin<hmin2*0.4 || hmin>hmin2*1.6)) return false;
		if((h<h2*0.6 || h>h2*1.4)) return false;
		return true;
}

//computes lbp values of input images and write it in to hist
void get_lbp_values(cv::Mat img0, cv::Mat img1, cv::Mat img2, cv::Mat img3, double *hist1, double *hist2, double *hist3, double *hist4)
{
	create_lbp lbp = create_lbp();
	lbp.create_lbp_class(img0, 1, 8, false, hist1);
	lbp.create_lbp_class(img1, 1, 8, false, hist2);
	lbp.create_lbp_class(img2, 1, 8, false, hist3);
	lbp.create_lbp_class(img3, 1, 8, false, hist4);
}

bool check_hsv_two_rect(cv::Rect roi1, cv::Rect roi2, cv::Mat img)
{

	int x11 = roi1.x;
	int x12 = roi1.x + roi1.width;
	int x21 = roi2.x;
	int x22 = roi2.x + roi2.width;

	int y11 = roi1.y;
	int y12 = roi1.y + roi1.height;
	int y21 = roi2.y;
	int y22 = roi2.y + roi2.height;

	//get HSV value
		cv::Mat hsv;
		cv::cvtColor(img, hsv, CV_BGR2HSV);
		int h1 = 0;
		int s1 = 0;
		int v1 = 0;
		int h2 = 0;
		int s2 = 0;
		int v2 = 0;
		int hmin1 = 0;
		int smin1 = 255;
		int vmin1 = 255;
		int hmax1 = 0;
		int smax1 = 0;
		int vmax1 = 0;
		int hmin2 = 0;
		int smin2 = 255;
		int vmin2 = 255;
		int hmax2 = 0;
		int smax2 = 0;
		int vmax2 = 0;

		for(int i=x11;i<x12;i++)
		{
			for(int j=y11;j<y12;j++)
			{
				if(hmin1 > hsv.at<cv::Vec3b>(j,i)[0]) hmin1 = hsv.at<cv::Vec3b>(j,i)[0];
				if(hmax1 < hsv.at<cv::Vec3b>(j,i)[0]) hmax1 = hsv.at<cv::Vec3b>(j,i)[0];
				if(smin1 > hsv.at<cv::Vec3b>(j,i)[1]) smin1 = hsv.at<cv::Vec3b>(j,i)[1];
				if(smax1 < hsv.at<cv::Vec3b>(j,i)[1]) smax1 = hsv.at<cv::Vec3b>(j,j)[1];
				if(vmin1 > hsv.at<cv::Vec3b>(j,i)[2]) vmin1 = hsv.at<cv::Vec3b>(j,j)[2];
				if(vmax1 < hsv.at<cv::Vec3b>(j,i)[2]) vmax1 = hsv.at<cv::Vec3b>(j,j)[2];
				h1 = h1 + hsv.at<cv::Vec3b>(j,i)[0];
				s1 = s1 + hsv.at<cv::Vec3b>(j,i)[1];
				v1 = v1 + hsv.at<cv::Vec3b>(j,i)[2];
			}
		}
		h1 = h1/(roi1.height*roi1.width);
		s1 = s1/(roi1.height*roi1.width);
		v1 = v1/(roi1.height*roi1.width);
		for(int i=x21;i<x22;i++)
		{
			for(int j=y21;j<y22;j++)
			{
				if(hmin2 > hsv.at<cv::Vec3b>(j,i)[0]) hmin2 = hsv.at<cv::Vec3b>(j,i)[0];
				if(hmax2 < hsv.at<cv::Vec3b>(j,i)[0]) hmax2 = hsv.at<cv::Vec3b>(j,i)[0];
				if(smin2 > hsv.at<cv::Vec3b>(j,i)[1]) smin2 = hsv.at<cv::Vec3b>(j,i)[1];
				if(smax2 < hsv.at<cv::Vec3b>(j,i)[1]) smax2 = hsv.at<cv::Vec3b>(j,i)[1];
				if(vmin2 > hsv.at<cv::Vec3b>(j,i)[2]) vmin2 = hsv.at<cv::Vec3b>(j,i)[2];
				if(vmax2 < hsv.at<cv::Vec3b>(j,i)[2]) vmax2 = hsv.at<cv::Vec3b>(j,i)[2];
				h2 = h2 + hsv.at<cv::Vec3b>(j,i)[0];
				s2 = s2 + hsv.at<cv::Vec3b>(j,i)[1];
				v2 = v2 + hsv.at<cv::Vec3b>(j,i)[2];
//				double a = hsv.at<cv::Vec3b>(j,i)[0];
//				double b = hsv.at<cv::Vec3b>(j,i)[1];
//				double c = hsv.at<cv::Vec3b>(j,i)[2];
//				std::cout << a<<" "<<b<<" "<<c<<" ";

			}
		}
		h2 = h2/(roi2.height*roi2.width);
		s2 = s2/(roi2.height*roi2.width);
		v2 = v2/(roi2.height*roi2.width);

//		std::cout<<"hsv ende ";
//		std::cout <<"  " << h1 << "h1 " << h2 << "h2 " << std::endl;
//		std::cout <<"  " << s1 << "s1 " << s2 << "s2 " <<std::endl;
//
//		if((hmax1*1.1<hmax2 || hmax1*0.9>hmax2) && h1!=0)
//		{
//			if((hmin1*1.1<hmin2 || hmin1*0.9>hmin2) && h1!=0)
//			{
//				if((h1*1.1<h2 || h1*0.9>h2) && h1!=0) return true;
//			}
//		}

//		if((hmax1-hmin1<h1*0.1 || hmax1-hmin1 >h1*4) && h1!=0) return false;
//		if((smax1-smin1<(s1*0.1) || smax1-smin1 >(s1*4)) && s1!=0 ) return true;
		if(vmax1-vmin1<(v1*0.1) || vmax1-vmin1 >(v1*2)) return false;

//		if((h1*2<h2 || h1*0.5>h2) && h1!=0 && h2!=0) return false;
////		if((smax1-smin1)<20 || (smax2-smin2)<200)return false;
//		if(pow((hmax1*hmax2),2)<20 || (hmin1-hmin2)<200)return false;
//		if((s1*2<s2 || s1*0.5>s2) && s1!=0 && s2!=0 ) return false;
//		if(vmax-vmin<(v*0.1) || vmax-vmin >(v*2)) return false;
		return true;
}

bool hsv_test(cv::Rect r1, cv::Rect r2, cv::Rect r3, cv::Rect r4, cv::Mat img)
{
	bool hsv = false;
//	if(!check_hsv_two_rect(r1, r2, img))hsv=true;
//	if(!check_hsv_two_rect(r1, r3, img))hsv=true;
//	if(!check_hsv_two_rect(r1, r4, img))hsv=true;
//	if(!check_hsv_two_rect(r2, r3, img))hsv=true;
//	if(!check_hsv_two_rect(r2, r4, img))hsv=true;
//	if(!check_hsv_two_rect(r3, r4, img))hsv=true;
//	if(!check_hsv(img(r1)))hsv=true;
//	if(!check_hsv(img(r2)))hsv=true;
//	if(!check_hsv(img(r3)))hsv=true;
//	if(!check_hsv(img(r4)))hsv=true;
	return hsv;
}
bool varianz(cv::Mat img, cv::Mat img2)
{
	double rows = img.rows;
	double cols = img.cols;
	double point_number = rows*cols*3;
	double r[255];
	double g[255];
	double b[255];
	double r_sum, g_sum, b_sum;
	double r_var=0;
	double g_var=0;
	double b_var=0;
//	double var=0;
	double r_mean=0;
	double g_mean=0;
	double b_mean=0;

	for(int i=0;i<255;i++)
	{
		r[i]=0;
		g[i]=0;
		b[i]=0;
	}
	cv::Mat hsv;
	cv::cvtColor(img, hsv, CV_BGR2HSV);
	cv::Mat hsv2;
	cv::cvtColor(img2, hsv2, CV_BGR2HSV);

	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)
		{
//			r[hsv.at<cv::Vec3b>(i,j)[0]]++;
//			g[hsv.at<cv::Vec3b>(i,j)[1]]++;
//			b[hsv.at<cv::Vec3b>(i,j)[2]]++;
//			r[hsv2.at<cv::Vec3b>(i,j)[0]]++;
//			g[hsv2.at<cv::Vec3b>(i,j)[1]]++;
//			b[hsv2.at<cv::Vec3b>(i,j)[2]]++;
			r[img.at<cv::Vec3b>(i,j)[0]]++;
			g[img.at<cv::Vec3b>(i,j)[1]]++;
			b[img.at<cv::Vec3b>(i,j)[2]]++;
			r[img2.at<cv::Vec3b>(i,j)[0]]++;
			g[img2.at<cv::Vec3b>(i,j)[1]]++;
			b[img2.at<cv::Vec3b>(i,j)[2]]++;
			r_sum = r_sum+hsv.at<cv::Vec3b>(i,j)[0];
			g_sum = g_sum+hsv.at<cv::Vec3b>(i,j)[1];
			b_sum = b_sum+img.at<cv::Vec3b>(i,j)[2];
		}
	}
	for(int i=0;i<255;i++)
	{
		r[i]=r[i]/point_number;
		g[i]=g[i]/point_number;
		b[i]=b[i]/point_number;
//		if(r[i]>=1)std::cout << r[i]<< "r "<<point_number<<"pummer ";
	}
	for(int i=0;i<255;i++)
	{
		r_mean=r_mean + r[i]*i;
		g_mean=g_mean + g[i]*i;
		b_mean=b_mean + b[i]*i;
//		if(r_mean!=r_mean)std::cout << i<<"i "<< r_mean<<"mean "<<r[i]<<"ri ";
	}
//	r_mean=r_mean / point_number;
//	g_mean=g_mean / point_number;
//	b_mean=b_mean / point_number;
	for(int i=0;i<255;i++)
	{
		r_var=r_var + pow((i),2)*r[i];
		g_var=g_var + pow((i-g_mean),2)*g[i];
		b_var=b_var + pow((i-b_mean),2)*b[i];

	}
	r_var=r_var-pow(r_mean, 2);
	r_var=sqrt(r_var);
	g_var=g_var-pow(g_mean, 2);
//	g_var=sqrt(g_var);
	b_var=b_var-pow(b_mean, 2);
//	b_var=sqrt(b_var);
//	std::cout <<r_var<<"rvar "<< r_mean<<"mean "<<std::endl;
//	r_mean=r_sum/point_number;
//	g_mean=g_sum/point_number;
//	b_mean=b_sum/point_number;
//
//		r_var = pow(r_sum-r_mean,2)/point_number;
//		g_var = pow(g_sum-r_mean,2)/point_number;
//		b_var = pow(b_sum-r_mean,2)/point_number;

//	std::cout<< r_var<<"rvar "<<g_var<<"gvar "<<b_var<<"b_var ";
//	var = (r_var);
//	std::cout << var <<"var ";

	if(r_var+g_var+b_var>1000)return true;
//	else if(g_var >10000)return true;
	else return false;
}

region* get_region(std::vector<int> &id, region &n)
{
	if(id.size()<1)
	{
		return &n;
	}else
	{
		int adress = id.back();
		id.pop_back();
		if(n.childs.size()>1)
		{
			return get_region(id, n.childs[adress-1]);
		}else{
			std::cout<<"Error: root does not exist";
			return NULL;
		}
	}
}

void convert_id(int id_int, std::vector<int> &id_vec)
{
	int count=0;
	int num = id_int;
	do ++count; while(num/=10);
	for(int i = 0; i<count; i++)
			  {
				  if(i==0)id_vec.push_back(id_int%10);
				  else id_vec.push_back((((id_int%static_cast<int>((pow(10,i+1)))-id_int%static_cast<int>((pow(10,i))))/static_cast<int>(pow(10,i)))));
			  }
}

region split(cv::Mat image, cv::Rect roi, double *lbp, int id) {
    std::vector<region> childs;
    region rs;
    rs.id = id;
    rs.roi = roi;
    rs.validity = true;
    rs.lbp_set = true;
    double rows_r, cols_r;
    rs.class_num=0;
    //Set LBP values
    for(int i = 0;i<10;i++)
    {
    	rs.lbp.push_back(lbp[i]);
    }
    //Get Size of new roi
    double rows = floor(image.rows/2);
    double cols = floor(image.cols/2);
    if(((static_cast<int>(image.rows))%2) > 0)
    {
    	rows_r = rows +1;
    }else
    {
    	rows_r = rows;
    }
    if(((static_cast<int>(image.cols))%2) > 0)
    {
    	cols_r = cols +1;
    }else
    {
    	cols_r = cols;
    }
    //Exit if window is to small
    if(rows < 3 || cols <3)
    {
    	return rs;
    }
    //Set roi and create new small image
    std::vector< std::vector<double> > lbp_values;
    lbp_values.resize(4);
    lbp_values[0].resize(10);
    lbp_values[1].resize(10);
    lbp_values[2].resize(10);
    lbp_values[3].resize(10);
    double lbp_values1[10];
    double lbp_values2[10];
    double lbp_values3[10];
    double lbp_values4[10];
   cv::Rect roi_1 = cv::Rect(0, 0, cols, rows);
   cv::Rect roi_2 = cv::Rect(cols, 0, cols_r, rows);
   cv::Rect roi_3 = cv::Rect(0, rows, cols, rows_r);
   cv::Rect roi_4 = cv::Rect(cols, rows, cols_r, rows_r);
   cv::Mat image1 = image(roi_1);
   cv::Mat image2 = image(roi_2);
   cv::Mat image3 = image(roi_3);
   cv::Mat image4 = image(roi_4);
   //get lbp value of the 4 new images
   get_lbp_values(image1, image2, image3, image4, lbp_values1,lbp_values2,lbp_values3,lbp_values4);
   for(int l=0;l<10;l++)
   {
	   lbp_values[0][l]=lbp_values1[l];
	   lbp_values[1][l]=lbp_values2[l];
	   lbp_values[2][l]=lbp_values3[l];
	   lbp_values[3][l]=lbp_values4[l];
   }
   //check if segment has to get split

//   double split_int = lbp_check(lbp_values);

//   bool split_now = test(lbp_values1, lbp_values2,lbp_values3,lbp_values4);
   //chisquare test
//   bool val = lbp_check_chi(lbp_values1, lbp_values2,lbp_values3,lbp_values4, (rows_r+rows)*(cols_r+cols));
//   	 double val = lbp_check_chi_four(lbp_values1, lbp_values4, lbp_values2, lbp_values3,(rows_r+rows)*(cols_r+cols));
//   	 double val = lbp_check_chi(lbp_values, (rows_r+rows)*(cols_r+cols), false);
   	 cv::Mat color_dev1,color_dev2, color_dev3, color_dev4;
   	cv::Mat color_mean1,color_mean2, color_mean3, color_mean4;
   	cv::meanStdDev(image1,  color_mean1, color_dev1);
   	cv::meanStdDev(image2,  color_mean2, color_dev2);
   	cv::meanStdDev(image3,  color_mean3, color_dev3);
   	cv::meanStdDev(image4,  color_mean4, color_dev4);

   	double meandevr=0;
	double meandevg=0;
	double meandevb=0;

   		meandevr = meandevr + color_dev1.at<double>(0) +color_dev2.at<double>(0) +color_dev3.at<double>(0) +color_dev4.at<double>(0);
   		meandevg = meandevg + color_dev1.at<double>(1) +color_dev2.at<double>(1) +color_dev3.at<double>(1) +color_dev4.at<double>(1);
   		meandevb = meandevb + color_dev1.at<double>(2) +color_dev2.at<double>(2) +color_dev3.at<double>(2) +color_dev4.at<double>(2);



//   	std::cout<<meandevr<<"r"<<std::endl;
//   	std::cout<<meandevg<<"g"<<std::endl;
//   	std::cout<<meandevb<<"b"<<std::endl;
//   	 bool split_now=true;
//if(val==val2)std::cout << "!!!";
//else std::cout<<val<<"_"<<val2<< "err";
//   	 if(val <= splitval)//split_now=false;   // 5.1 good value
//	if(meandevr <60 || meandevg <60 || meandevb <60)
//	{
//		if(val <= 5.1)split_now=false;
//		split_now=false;
//	}else{
//		if(val <= 3.1)split_now=false;
//	}
//   	 if(val < 6 && val >4.5)split_now = hsv_test(roi_1, roi_2, roi_3, roi_4, image);
//   	 if(val < 6 && val >4.5)split_now = varianz(image);

    if(roi.height>8 && roi.width>8)//(split_now)//
    {
    	rs.lbp_set = false;
        region r1 = split(image1, cv::Rect(roi.x, roi.y, cols,rows), lbp_values1, (rs.id*10)+1);
        region r2 = split(image2, cv::Rect(roi.x+cols, roi.y, cols_r,rows), lbp_values2, (rs.id*10)+2);
        region r3 = split(image3, cv::Rect(roi.x, roi.y+rows, cols,rows_r), lbp_values3, (rs.id*10)+3);
        region r4 = split(image4, cv::Rect(roi.x+cols, roi.y+rows, cols_r,rows_r), lbp_values4, (rs.id*10)+4);
        rs.childs.push_back( r1 );
        rs.childs.push_back( r2 );
        rs.childs.push_back( r3 );
        rs.childs.push_back( r4 );
        validity[rs.id]=false;
    }else{
    	validity[rs.id]=true;
    }

    return rs;
}

void neighbor_graph(region rs, std::vector<int> N, std::vector<int> E, std::vector<int> S, std::vector<int> W, int splitpos) {

	int id =rs.id;

    if(splitpos >0)
    {
		for(int num=0;num<2;num++)
		{
			std::vector<int> swap, result;

			if(splitpos == 1 && num==0)
					swap = N;
			if(splitpos == 1 && num==1)
					swap = W;

			if(splitpos == 2 && num==0)
					swap = N;
			if(splitpos == 2 && num==1)
					swap = E;

			if(splitpos == 3 && num==0)
					swap = S;
			if(splitpos == 3 && num==1)
					swap = W;

			if(splitpos == 4 && num==0)
					swap = E;
			if(splitpos == 4 && num==1)
					swap = S;


//			std::cout<<"firsttt"<<std::endl;


			for(size_t i=0;i<swap.size();i++)
				{
//				std::cout<<"seconddd"<<std::endl;
						std::vector<int> idN;
						convert_id(swap[i], idN);
						region *nN = get_region(idN, r);
						region &n1 = *nN;

						std::vector<int> neighbor_;

						if(neighbor(rs, n1))
						{

//							std::cout<<"neighborfound"<<std::endl;
							result.push_back(n1.id);
							if(splitpos == 1 && num==0)
								neighbor_ = n1.S;
							if(splitpos == 1 && num==1)
								neighbor_ = n1.E;

							if(splitpos == 2 && num==0)
								neighbor_ = n1.S;
							if(splitpos == 2 && num==1)
								neighbor_ = n1.W;

							if(splitpos == 3 && num==0)
								neighbor_ = n1.N;
							if(splitpos == 3 && num==1)
								neighbor_ = n1.E;

							if(splitpos == 4 && num==0)
								neighbor_ = n1.W;
							if(splitpos == 4 && num==1)
								neighbor_ = n1.N;

							for(size_t j=0;j<neighbor_.size(); j++)
							{
								if(neighbor_[j]== (id-(id%10))/10)
								{
									neighbor_.push_back(id);
								}
							}

							if(splitpos == 1 && num==0)
								n1.S = neighbor_;
							if(splitpos == 1 && num==1)
								n1.E = neighbor_;

							if(splitpos == 2 && num==0)
								n1.S = neighbor_;
							if(splitpos == 2 && num==1)
								n1.W = neighbor_;

							if(splitpos == 3 && num==0)
								n1.N = neighbor_;
							if(splitpos == 3 && num==1)
								n1.E = neighbor_;

							if(splitpos == 4 && num==0)
								n1.W = neighbor_;
							if(splitpos == 4 && num==1)
								n1.N = neighbor_;
						}
				}
//				N=swap;


			if(splitpos == 1 && num==0)
					rs.N = result;
			if(splitpos == 1 && num==1)
					rs.W = result;

			if(splitpos == 2 && num==0)
					rs.N = result;
			if(splitpos == 2 && num==1)
					rs.E = result;

			if(splitpos == 3 && num==0)
					rs.S = result;
			if(splitpos == 3 && num==1)
					rs.W = result;

			if(splitpos == 4 && num==0)
					rs.E = result;
			if(splitpos == 4 && num==1)
					rs.S = result;

		}

    }


    std::vector<int> N1_, N2_, E2_, E4_, S3_, S4_, W1_, W3_;
    E2_.push_back((rs.id*10)+2);
    S3_.push_back((rs.id*10)+3);
    S4_.push_back((rs.id*10)+4);
    W1_.push_back((rs.id*10)+1);
    N1_.push_back((rs.id*10)+1);
    E4_.push_back((rs.id*10)+4);
    N2_.push_back((rs.id*10)+2);
    W3_.push_back((rs.id*10)+3);

//  	std::cout<<rs.childs.size()<<"childsize"<<std::endl;
    if(rs.childs.size()>1)
    {
//    	std::cout<<"jetzt"<<std::endl;



    		std::vector<int> idN1;
    		convert_id(rs.childs[0].id, idN1);
    		region *nN1 = get_region(idN1, r);
    		region &n1 = *nN1;
    		std::vector<int> idN2;
    		convert_id(rs.childs[1].id, idN2);
    		region *nN2 = get_region(idN2, r);
    		region &n2 = *nN2;
    		std::vector<int> idN3;
    		convert_id(rs.childs[2].id, idN3);
    		region *nN3 = get_region(idN3, r);
    		region &n3 = *nN3;
    		std::vector<int> idN4;
    		convert_id(rs.childs[3].id, idN4);
    		region *nN4 = get_region(idN4, r);
    		region &n4 = *nN4;

    			n1.E=E2_;
    			n1.S=S3_;
    			n1.N=rs.N;
    			n1.W=rs.W;

    			n2.E=rs.E;
    			n2.S=S4_;
    			n2.N=rs.N;
    			n2.W=W1_;

      			n3.E=E4_;
      			n3.S=rs.S;
        		n3.N=N1_;
        		n3.W=rs.W;

      			n4.E=rs.E;
        		n4.S=rs.S;
        		n4.N=N2_;
        		n4.W=W3_;




//    	rs.childs[0].E=E2_;
//    	rs.childs[0].S=S3_;
//    	rs.childs[1].S=S4_;
//    	rs.childs[1].W=W1_;
//    	rs.childs[2].E=E4_;
//    	rs.childs[2].N=N1_;
//    	rs.childs[3].N=N2_;
//    	rs.childs[3].W=W3_;
//      	rs.childs[0].N=rs.N;
//        rs.childs[0].W=rs.W;
//        rs.childs[1].N=rs.N;
//        rs.childs[1].E=rs.E;
//        rs.childs[2].S=rs.S;
//        rs.childs[2].W=rs.W;
//        rs.childs[3].E=rs.E;
//        rs.childs[3].S=rs.S;
    	neighbor_graph(n1, rs.N, E2_ , S3_, rs.W,  1);
    	neighbor_graph(n2, rs.N, rs.E, S4_, W1_, 2);
    	neighbor_graph(n3, N1_, E4_,rs.S, rs.W, 3);
    	neighbor_graph(n4, N2_, rs.E, rs.S, W3_, 4);
    }
    return;
}



void clear_neighbor(region &reg)
{
	//Erase similar entries
//	int size = reg.neighbors.size();
	for(uint i=0;i<reg.neighbors.size();i++)
	{
		for(uint j=i+1;j<reg.neighbors.size();j++)
		{
			if(reg.neighbors[i]==reg.neighbors[j])
			{
				reg.neighbors.erase(reg.neighbors.begin()+j);
			}
		}
	}
	if(reg.childs.size()>1)
	{
		clear_neighbor(reg.childs[0]);
		clear_neighbor(reg.childs[1]);
		clear_neighbor(reg.childs[2]);
		clear_neighbor(reg.childs[3]);
	}
}

void get_border_row(region &n, region &o)
{
	if(n.childs.size()>1 && (o.id%10==1 ||o.id%10==3))
	{
		get_border_row(n.childs[0], o);
		get_border_row(n.childs[2], o);
	}
	else if(n.childs.size()>1 && (o.id%10==2 ||o.id%10==4))
	{
		get_border_row(n.childs[1], o);
		get_border_row(n.childs[3], o);
	}else if(n.childs.size()<1)
	{
		if(neighbor(n,o)&& n.id!=o.id)
		{
			o.neighbors.push_back(n.id);
			n.neighbors.push_back(o.id);
		}

	}
}
void get_border_row_sameroot(region &n, region &o)
{
	if(n.childs.size()>1 && (o.id%10==1 ||o.id%10==3))
	{
		get_border_row_sameroot(n.childs[1], o);
		get_border_row_sameroot(n.childs[3], o);
	}
	else if(n.childs.size()>1 && (o.id%10==2 ||o.id%10==4))
	{
		get_border_row_sameroot(n.childs[0], o);
		get_border_row_sameroot(n.childs[2], o);
	}else if(n.childs.size()<1)
	{
		if(neighbor(n,o)&& n.id!=o.id)
		{
			o.neighbors.push_back(n.id);
			n.neighbors.push_back(o.id);
		}

	}
}
void get_border_col(region &n, region &o)
{
	if(n.childs.size()>1 && (o.id%10==3 ||o.id%10==4))
	{
		get_border_col(n.childs[2], o);
		get_border_col(n.childs[3], o);
	}
	else if(n.childs.size()>1 && (o.id%10==1 ||o.id%10==2))
	{
		get_border_col(n.childs[0], o);
		get_border_col(n.childs[1], o);
	}else if(n.childs.size()<1)
	{
		if(neighbor(n,o)&& n.id!=o.id)
		{
			o.neighbors.push_back(n.id);
			n.neighbors.push_back(o.id);
		}
	}
}

void get_border_col_sameroot(region &n, region &o)
{
	if(n.childs.size()>1 && (o.id%10==3 ||o.id%10==4))
	{
		get_border_col_sameroot(n.childs[1], o);
		get_border_col_sameroot(n.childs[0], o);
	}
	else if(n.childs.size()>1 && (o.id%10==1 ||o.id%10==2))
	{
		get_border_col_sameroot(n.childs[2], o);
		get_border_col_sameroot(n.childs[3], o);
	}else if(n.childs.size()<1)
	{
		if(neighbor(n,o)&& n.id!=o.id)
		{
			o.neighbors.push_back(n.id);
			n.neighbors.push_back(o.id);
		}
	}
}

void get_border_nextroot(region &n, region &o, int root)
{
	if(n.childs.size()>1 && (root==12 || root==34))
	{
		get_border_nextroot(n.childs[0], o, root);
		get_border_nextroot(n.childs[2], o, root);
	}
	else if(n.childs.size()>1 && (root==13 || root==24))
	{
		get_border_nextroot(n.childs[0], o,root);
		get_border_nextroot(n.childs[1], o,root);
	}else if(n.childs.size()>1 && (root==42 || root==31))
	{
		get_border_nextroot(n.childs[2], o, root);
		get_border_nextroot(n.childs[3], o, root);
	}else if(n.childs.size()>1 && (root==43 || root==21))
	{
		get_border_nextroot(n.childs[1], o, root);
		get_border_nextroot(n.childs[3], o, root);
	}else if(n.childs.size()<1)
	{
		if(neighbor(n, o)&& n.id!=o.id)
		{
//			std::cout<<"merging downsize done"<<std::endl;
			o.neighbors.push_back(n.id);
			n.neighbors.push_back(o.id);
		}
	}
}




void init_neighbour(region &center)
{
	  int count=0;
	  int num = center.id;
	  do ++count; while(num/=10);

	  std::cout<< center.id <<"id "<<count<<"count"<<std::endl;
	  std::vector<int> id_;
	  bool border_n = false;
	  bool border_e = false;
	  bool border_s = false;
	  bool border_w = false;
	  bool border_1 = false;
	  bool border_2 = false;
	  bool border_3 = false;
	  bool border_4 = false;

	  if(center.childs.size()<1)
	  {
		  int n1=0,n2=0,n3=0,n4=0;
		  for(int i = 0; i<count; i++)
		  {
			  if(i==0)id_.push_back(center.id%10);
			  else id_.push_back((((center.id%static_cast<int>((pow(10,i+1)))-center.id%static_cast<int>((pow(10,i))))/static_cast<int>(pow(10,i)))));
			  if(i<count-1){
				  if(id_[i]==1)n1++;
				  if(id_[i]==2)n2++;
				  if(id_[i]==3)n3++;
				  if(id_[i]==4)n4++;
			  }
		  }
		  if(id_[count-1]==1 && n1==0){
			  if(n2==0 && n3>0 && n4>0)border_3=true;
			  if(n3==0 && n2>0 && n4>0)border_2=true;
			  if(n2==0 && n3==0 && n4>0 )
				  {
					  border_2=true;
					  border_3=true;
				  }
			  if(n2==0 && n3>0 && n4==0 )border_3=true;
			  if(n2>0 && n3==0 && n4==0 )border_2=true;
		  }else if(id_[count-1]==2 && n2==0) {
			  if(n1>0 && n3>0 && n4==0)border_4=true;
			  if(n1==0 && n3>0 && n4>0)border_3=true;
			  if(n1==0 && n3>0 && n4==0 )
				  {
				  	  border_4=true;
				  	  border_3=true;
				  }
			  if(n1==0 && n3==0 && n4>0 )border_3=true;
			  if(n1>0 && n3==0 && n4==0 )border_4=true;
		  }else if(id_[count-1]==3 && n3==0) {
			  if(n1>0 && n2>0 && n4==0)border_1=true;
			  if(n1==0 && n2>0 && n4>0)border_2=true;
			  if(n1==0 && n2>0 && n4==0 )
				  {
				  	  border_1=true;
				  	  border_2=true;
				  }
			  if(n1==0 && n2==0 && n4>0 )border_2=true;
			  if(n1>0 && n2==0 && n4==0 )border_1=true;
		  }else if(id_[count-1]==4 && n4==0) {
			  if(n1>0 && n2>0 && n3==0)border_1=true;
		  	  if(n1>0 && n2==0 && n3>0)border_4=true;
		  	  if(n1>0 && n2==0 && n3==0 )
		  		  {
		  		  	  border_1=true;
		  		  	  border_4=true;
		  		  }
		  	  if(n1==0 && n2==0 && n3>0 )border_4=true;
		  	  if(n1==0 && n2>0 && n3==0 )border_1=true;
		  }
		  if(((n2>0 || n4>0) && n3==0 &&n1==0)&&(id_[count-1]==2||id_[count-1]==4))border_e=true;
		  if(((n3>0 || n4>0) && n2==0 &&n1==0)&&(id_[count-1]==3||id_[count-1]==4))border_s=true;
		  if(((n2>0 || n1>0) && n3==0 &&n4==0)&&(id_[count-1]==1||id_[count-1]==2))border_n=true;
		  if(((n1>0 || n1>0) && n2==0 &&n4==0)&&(id_[count-1]==1||id_[count-1]==3))border_w=true;
		  if(count==1)
		  {
			  if(center.id == 1 && center.id == 3){ center.neighbors.push_back(2);center.neighbors.push_back(4);}
			  if(center.id == 2 && center.id == 4){ center.neighbors.push_back(1);center.neighbors.push_back(3);}
		  }else
		  {
			  //Same tree
			  //get right neighbor
			  if(id_[0] == 1 || id_[0]==3)
			  {
					  std::vector<int> right = id_;
					  right[0] = right[0]+1;
					  region *neighbor_l = get_region(right, r);
					  region &neighbor_l_ = *neighbor_l;
					  get_border_row(neighbor_l_, center);
			  }
			  //get left neighbor
			  if(id_[0] == 2 || id_[0]==4)
			  {
					  std::vector<int> left = id_;
					  left[0] = left[0]-1;
					  region *neighbor_r = get_region(left, r);
//					  region &neighbor_r_ = *neighbor_r;
					  get_border_row((*neighbor_r), center);
			  }
			  //get upper neighbor
			  if(id_[0] == 3 || id_[0]==4)
			  {
					  std::vector<int> upper = id_;
					  upper[0] = upper[0]-2;
					  region *neighbor_u = get_region(upper, r);
					  region &neighbor_u_ = *neighbor_u;
					  get_border_col(neighbor_u_, center);
			  }
			  //get lower neighbor
			  if(id_[0] == 1 || id_[0]==2)
			  {
					  std::vector<int> lower = id_;
					  lower[0] = lower[0]+2;
					  region *neighbor_l = get_region(lower, r);
					  region &neighbor_l_ = *neighbor_l;
					  get_border_col(neighbor_l_, center);
			  }
			  //Same tree, neighbor in higher node
			  //get right neighbor
			  if(id_[0] == 2 || id_[0]==4)
			  {
				  if(!border_2&&!border_e)
				  {
					  std::vector<int> right2 = id_;
					  right2.erase(right2.begin());
					  right2[0] = right2[0]+1;
					  if(right2[0]<5)
					  {
						  region *neighbor_r2 = get_region(right2, r);
						  if(neighbor_r2!=NULL){
						  region &neighbor_r2_ =*neighbor_r2;
						  get_border_row_sameroot(neighbor_r2_, center);
						  }
					  }
				  }
			  }
			  //get left neighbor
			  if(id_[0] == 1 || id_[0]==3)
			  {
				  if(!border_4&&!border_w)
				  {
					  std::vector<int> left2 = id_;
					  left2.erase(left2.begin());
					  left2[0] = left2[0]-1;
					  if(left2[0]>0)
					  {
						  region *neighbor_l2 = get_region(left2, r);
						  if(neighbor_l2!=NULL){
						  region &neighbor_l2_ = *neighbor_l2;
						  get_border_row_sameroot(neighbor_l2_, center);
						  }
					  }
				  }
			  }
			  //get upper neighbor
			  if(id_[0] == 1 || id_[0]==2)
			  {
				  if(!border_n && !border_1)
				  {
					  std::vector<int> upper = id_;
					  upper.erase(upper.begin());
					  upper[0] = upper[0]-2;
					  if(upper[0]>0)
					  {
						  region *neighbor_u = get_region(upper, r);
						  region &neighbor_u_ = *neighbor_u;
						  get_border_col_sameroot(neighbor_u_, center);
					  }
				  }
			  }
			  //get lower neighbor
			  if(id_[0] == 3 || id_[0]==4)
			  {
				  if(!border_s && !border_3)
				  {
					  std::vector<int> lower = id_;
					  lower.erase(lower.begin());
					  lower[0] = lower[0]+2;
					  if(lower[0]<5)
					  {
						  region *neighbor_l = get_region(lower, r);
						  region &neighbor_l_ = *neighbor_l;
						  get_border_col_sameroot(neighbor_l_, center);
						  }
					  }
			  }
			  //Different root
//				  if(id_[count-1]==1)
//				  {
//					  if(border_2)
//					  {
//						  get_border_nextroot(r.childs[1], center, 12);
//					  }if(border_3)
//					  {
//						  get_border_nextroot(r.childs[2], center, 13);
//					  }
//				  }if(id_[count-1]==2)
//				  {
//					  if(border_4)
//					  {
//						  get_border_nextroot(r.childs[0], center, 21);
//					  }if(border_3)
//					  {
//						  get_border_nextroot(r.childs[3], center, 24);
//					  }
//				  }if(id_[count-1]==3)
//				  {
//					  if(border_1)
//					  {
//						  get_border_nextroot(r.childs[0], center, 31);
//					  }if(border_2)
//					  {
//						  get_border_nextroot(r.childs[3], center, 34);
//					  }
//				  }if(id_[count-1]==4)
//				  {
//					  if(border_1)
//					  {
//						  get_border_nextroot(r.childs[1], center, 42);
//					  }if(border_4)
//					  {
//						  get_border_nextroot(r.childs[2], center, 43);
//					  }
//				  }
		  }

		  if(id_[count-1]==1)
		  {
			  std::vector<int> test = id_;
			  std::vector<int> test2 = id_;
			  test[count-1]=3;
			  test2[count-1]=2;
				region *n1s = get_region(test, r);
				region *n2s = get_region(test2, r);
				region &n1 = *n1s;
				region &n2 = *n2s;
				if(n1.childs.size()>1)
				{
					int uebergabe=0;
					convert_id(uebergabe, test);
//					center.neighbors.push_back(uebergabe);
				}
				if(n2.childs.size()>1)
				{
					int uebergabe=0;
					convert_id(uebergabe, test2);
//					center.neighbors.push_back(uebergabe);
				}

		  }if(id_[count-1]==2)
		  {

			  std::vector<int> test = id_;
			  std::vector<int> test2 = id_;
			  test[count-1]=1;
			  test2[count-1]=4;
				region *n1s = get_region(test, r);
				region *n2s = get_region(test2, r);
				region &n1 = *n1s;
				region &n2 = *n2s;
				if(n1.childs.size()>1)
				{
					int uebergabe;
					convert_id(uebergabe, test);
					center.neighbors.push_back(uebergabe);
				}
				if(n2.childs.size()>1)
				{
					int uebergabe;
					convert_id(uebergabe, test2);
					center.neighbors.push_back(uebergabe);
				}

		  }if(id_[count-1]==3)
		  {
			  std::vector<int> test = id_;
			  std::vector<int> test2 = id_;
			  test[count-1]=1;
			  test2[count-1]=4;
				region *n1s = get_region(test, r);
				region *n2s = get_region(test2, r);
				region &n1 = *n1s;
				region &n2 = *n2s;
				if(n1.childs.size()>1)
				{
					int uebergabe;
					convert_id(uebergabe, test);
					center.neighbors.push_back(uebergabe);
				}
				if(n2.childs.size()>1)
				{
					int uebergabe;
					convert_id(uebergabe, test2);
					center.neighbors.push_back(uebergabe);
				}

		  }if(id_[count-1]==4)
		  {
			  std::vector<int> test = id_;
			  std::vector<int> test2 = id_;
			  test[count-1]=3;
			  test2[count-1]=2;
				region *n1s = get_region(test, r);
				region *n2s = get_region(test2, r);
				region &n1 = *n1s;
				region &n2 = *n2s;
				if(n1.childs.size()<1)
				{
					int uebergabe;
					convert_id(uebergabe, test);
					center.neighbors.push_back(uebergabe);
				}
				if(n2.childs.size()<1)
				{
					int uebergabe;
					convert_id(uebergabe, test2);
					center.neighbors.push_back(uebergabe);
				}
		  }

	  }
	  else if(center.childs.size()>1)
	  {
		  init_neighbour(center.childs[0]);
		  init_neighbour(center.childs[1]);
		  init_neighbour(center.childs[2]);
		  init_neighbour(center.childs[3]);
	  }


}

void draw_rect(cv::Mat img, region r) {
    for(int i = r.roi.y; i<r.roi.y+r.roi.height;i++)
    {
    	for(int rgb = 0;rgb<3;rgb++)
    	{
    		img.at<cv::Vec3b>(i,r.roi.x)[rgb]=130;
    		img.at<cv::Vec3b>(i,r.roi.x+r.roi.width-1)[rgb]=130;
    	}
   	}
    for(int i = r.roi.x; i<r.roi.x+r.roi.width-1;i++)
    {
    	for(int rgb = 0;rgb<3;rgb++)
    	{
    		img.at<cv::Vec3b>(r.roi.y,i)[rgb]=130;
    		img.at<cv::Vec3b>(r.roi.y+r.roi.height-1,i)[rgb]=130;
    	}
    }
    for(uint i=0; i<r.childs.size(); i++) {
        draw_rect(img, r.childs[i]);
    }
}

bool check_hsv_two_region(region &r1, region &r2, cv::Mat img)
{

	int x11 = r1.roi.x;
	int x12 = r1.roi.x + r1.roi.width;
//	int x21 = r2.roi.x;
//	int x22 = r2.roi.x + r2.roi.width;

	int y11 = r1.roi.y;
//	int y12 = r1.roi.y + r1.roi.height;
//	int y21 = r2.roi.y;
	int y22 = r2.roi.y + r2.roi.height;

	//get HSV value
		cv::Mat hsv;
		cv::cvtColor(img, hsv, CV_BGR2HSV);
		int h1 = 0;
		int s1 = 0;
		int v1 = 0;
		int h2 = 0;
		int s2 = 0;
		int v2 = 0;
		int hmin1 = 0;
		int smin1 = 255;
		int vmin1 = 255;
		int hmax1 = 0;
		int smax1 = 0;
		int vmax1 = 0;
		int hmin2 = 0;
		int smin2 = 255;
		int vmin2 = 255;
		int hmax2 = 0;
		int smax2 = 0;
		int vmax2 = 0;

		for(int i=x11;i<x12;i++)
		{
			for(int j=y11;j<y22;j++)
			{
				if(hmin1 > hsv.at<cv::Vec3b>(i,j)[0]) hmin1 = hsv.at<cv::Vec3b>(i,j)[0];
				if(hmax1 < hsv.at<cv::Vec3b>(i,j)[0]) hmax1 = hsv.at<cv::Vec3b>(i,j)[0];
				if(smin1 > hsv.at<cv::Vec3b>(i,j)[1]) smin1 = hsv.at<cv::Vec3b>(i,j)[1];
				if(smax1 < hsv.at<cv::Vec3b>(i,j)[1]) smax1 = hsv.at<cv::Vec3b>(i,j)[1];
				if(vmin1 > hsv.at<cv::Vec3b>(i,j)[2]) vmin1 = hsv.at<cv::Vec3b>(i,j)[2];
				if(vmax1 < hsv.at<cv::Vec3b>(i,j)[2]) vmax1 = hsv.at<cv::Vec3b>(i,j)[2];
				h1 = h1 + hsv.at<cv::Vec3b>(i,j)[0];
				s1 = s1 + hsv.at<cv::Vec3b>(i,j)[1];
				v1 = v1 + hsv.at<cv::Vec3b>(i,j)[2];
			}
		}
		h1 = h1/(r1.roi.height*r1.roi.width);
		s1 = s1/(r1.roi.height*r1.roi.width);
		v1 = v1/(r1.roi.height*r1.roi.width);
		for(int i=x11;i<x12;i++)
		{
			for(int j=y11;j<y22;j++)
			{
				if(hmin2 > hsv.at<cv::Vec3b>(i,j)[0]) hmin2 = hsv.at<cv::Vec3b>(i,j)[0];
				if(hmax2 < hsv.at<cv::Vec3b>(i,j)[0]) hmax2 = hsv.at<cv::Vec3b>(i,j)[0];
				if(smin2 > hsv.at<cv::Vec3b>(i,j)[1]) smin2 = hsv.at<cv::Vec3b>(i,j)[1];
				if(smax2 < hsv.at<cv::Vec3b>(i,j)[1]) smax2 = hsv.at<cv::Vec3b>(i,j)[1];
				if(vmin2 > hsv.at<cv::Vec3b>(i,j)[2]) vmin2 = hsv.at<cv::Vec3b>(i,j)[2];
				if(vmax2 < hsv.at<cv::Vec3b>(i,j)[2]) vmax2 = hsv.at<cv::Vec3b>(i,j)[2];
				h2 = h2 + hsv.at<cv::Vec3b>(i,j)[0];
				s2 = s2 + hsv.at<cv::Vec3b>(i,j)[1];
				v2 = v2 + hsv.at<cv::Vec3b>(i,j)[2];
			}
		}
		h2 = h2/(r2.roi.height*r2.roi.width);
		s2 = s2/(r2.roi.height*r2.roi.width);
		v2 = v2/(r2.roi.height*r2.roi.width);

//		std::cout<<"hsv ende ";
//		std::cout <<"  " << h << "h " << hmin << "hmin " << hmax << "hmax " << rows*cols << std::endl;
//		std::cout <<"  " << s << "s " << smin << "smin " << smax << "smax " <<"HSV CHECK DURCHGEFUEHRT" << std::endl;
//
//		if((hmax1*1.1<hmax2 || hmax1*0.9>hmax2) && h1!=0)
//		{
//			if((hmin1*1.1<hmin2 || hmin1*0.9>hmin2) && h1!=0)
//			{
//				if((h1*1.1<h2 || h1*0.9>h2) && h1!=0) return true;
//			}
//		}
		if((h1*1.2<h2 || h1*0.8>h2)) return true;
//		if((smax-smin<(s*0.6) || smax-smin >(s*1.1)) && s!=0 ) return true;
//		if(vmax-vmin<(v*0.1) || vmax-vmin >(v*2)) return false;
		return false;
}

void get_mean_color(region &reg, region_values &val, cv::Mat image)
{
	int x11 = reg.roi.x;
	int x12 = reg.roi.x + reg.roi.width;
	int y11 = reg.roi.y;
	int y12 = reg.roi.y + reg.roi.height;

	int r = 0;
	int g = 0;
	int b = 0;

	for(int i=x11;i<x12;i++)
	{
		for(int j=y11;j<y12;j++)
		{
			r = r + image.at<cv::Vec3b>(i,j)[2];
			g = g + image.at<cv::Vec3b>(i,j)[1];
			b = b + image.at<cv::Vec3b>(i,j)[0];
		}
	}
	r = r/(reg.roi.width*reg.roi.height);
	g = g/(reg.roi.width*reg.roi.height);
	b = b/(reg.roi.width*reg.roi.height);
	if(val.r==0&&val.g==0&&val.b==0)
	{
		val.r = r;
		val.g = g;
		val.b = b;
	}else{
		val.r = (val.r+r)/2;
		val.g = (val.g+g)/2;
		val.b = (val.b+b)/2;
	}
}
bool merge_lbp_chi(region& r1, region& r2, std::vector<region_values> &reg_val, cv::Mat image)
{
	double n1_lbp[10];
	double n2_lbp[10];
	double n3_lbp[10];
	double n4_lbp[10];
    if(r1.neighbors.size()>1) //&&reg_val[r1.class_num].members.size()>1)   <-----------------------------------------------Verbesserung?
	{
//		std::cout<< "pos1";
		std::vector<int> id1, id2, id3;
		convert_id(r1.neighbors[0], id1);
		convert_id(r1.neighbors[1], id2);
		region *n1s = get_region(id1, r);
		region *n2s = get_region(id2, r);
		region &n1 = *n1s;
		region &n2 = *n2s;
		for(int i=0;i<10;i++)
		{
			n1_lbp[i]= r1.lbp[i];
			n2_lbp[i]= r2.lbp[i];
			n3_lbp[i]= n1.lbp[i];
			n4_lbp[i]= n2.lbp[i];
		}

		std::vector< std::vector<double> > hist;
		hist.resize(2);
		for(int i=0;i<10;i++)
		{
			hist[0].push_back(r1.lbp[i]);
		}
		for(int i=0;i<10;i++)
		{
			hist[1].push_back(r2.lbp[i]);
		}
//		double lbp_val = lbp_check_chi(n1_lbp, n2_lbp, n3_lbp, n4_lbp, r1.roi.height*r1.roi.width);
		double lbp_val = lbp_check_chi(hist, r1.roi.height*r1.roi.width, false);
	   	bool split = false;
		if(lbp_val <= 10.5)split= true;
//	   	 if(lbp_val < 12 && lbp_val >10)split = hsv_test(r1.roi, r2.roi, n1.roi, n2.roi, image);
//	   	if(lbp_val < 13 && lbp_val >9)split = varianz(image(r1.roi), image(r2.roi));
//	   	split = varianz(image(r1.roi), image(r2.roi));
//	   	split = check_hsv(image(r1.roi), image(r2.roi));
	   	return split;

	}else if(reg_val[r1.class_num].members.size()>1)
	{
		std::cout<< "pos2";
		std::vector<int> id1, id2, id3;
		convert_id(reg_val[r1.class_num].members[0], id1);
		convert_id(reg_val[r1.class_num].members[1], id2);
		region *n1s = get_region(id1, r);
		region *n2s = get_region(id2, r);
		region &n1 = *n1s;
		region &n2 = *n2s;
		for(int i=0;i<10;i++)
		{
			n1_lbp[i]= r1.lbp[i];
			n2_lbp[i]= r2.lbp[i];
			n3_lbp[i]= n1.lbp[i];
			n4_lbp[i]= n2.lbp[i];
		}
		double lbp_val = lbp_check_chi_four(n1_lbp, n2_lbp, n3_lbp, n4_lbp, r1.roi.height*r1.roi.width);
		if(lbp_val < 4)return true; //4
		else if(lbp_val < 8 && lbp_val >4.5)return hsv_test(r1.roi, r2.roi, n1.roi, n2.roi, image); //8 4.5
		else return false;
	}else
	{
//		std::cout<< "pos3";
		for(uint i=0;i<r1.neighbors.size();i++)
		{
			std::vector<int> id;
			convert_id(reg_val[r1.class_num].members[i], id);
			region *ns = get_region(id, r);
			region &n = *ns;
			if(n.class_num == r1.class_num)
			{
				if(check_hsv_two_region(r1, n, image))return true;
			}
		}
	}
	return false;
}

bool group_region(region& r1, region& r2, std::vector<region_values> &region_val, cv::Mat &image, bool not_merged)
{
//	for(int i = 0; i<10;i++)
//					{
//						std::cout << r1.lbp[i]<<"--"<<i<<"--";
//					}std::cout<<"r1"<<std::endl;
//					for(int i = 0; i<10;i++)
//									{
//										std::cout << r2.lbp[i]<<"--"<<i<<"--";
//									}std::cout<<"r2"<<std::endl;
//	std::cout << r2.validity <<"r2valid "<< r1.lbp_set <<"r1lbpset "<<  r2.lbp_set<< "r2lbpset "<<std::endl;

//	double init[10];
//	cv::Rect newrect;
//		if(r1.roi.x == r2.roi.x)
//		{
//			if(r1.roi.y > r2.roi.y)
//			{
//				newrect = cv::Rect(r1.roi.x, r1.roi.y, r1.roi.width+r2.roi.width, r1.roi.height+r2.roi.height);
//			}else{
//				newrect = cv::Rect(r2.roi.x, r2.roi.y, r1.roi.width+r2.roi.width, r1.roi.height+r2.roi.height);
//			}
//		}else if(r1.roi.y == r2.roi.y)
//		{
//			if(r1.roi.x > r2.roi.x)
//			{
//				newrect = cv::Rect(r1.roi.x, r1.roi.y, r1.roi.width+r2.roi.width, r1.roi.height+r2.roi.height);
//			}else{
//				newrect = cv::Rect(r2.roi.x, r2.roi.y, r1.roi.width+r2.roi.width, r1.roi.height+r2.roi.height);
//			}
//		}
//



	   cv::Mat image1 = image(r1.roi);
	   cv::Mat image2 = image(r2.roi);
//	   cv::Mat color_dev1,color_dev2;
//	   cv::Mat color_mean1,color_mean2;
//	   cv::meanStdDev(image1,  color_mean1, color_dev1);
//	   cv::meanStdDev(image2,  color_mean2, color_dev2);
	   struct feature_results results1, results2;
	   color_parameter color = color_parameter();
	   if(!image1.empty() && !image2.empty())
	   {
		   color.get_color_parameter(image1, &results1);
		   color.get_color_parameter(image2, &results2);
	   }
	   if(not_merged)
		   region_val[r1.class_num].feat_res.push_back(results1);

	   double s_mean_m = 0;
	   double s_std_m = 0;
	   double v_mean_m = 0;
	   double v_std_m = 0;
	   double colorfulness_m = 0;
	   double dom_color_m = 0;
	   double dom_color_2_m = 0;

	   for(size_t i=0;i<region_val[r1.class_num].feat_res.size();i++)
	   {
		   s_mean_m = s_mean_m + region_val[r1.class_num].feat_res[i].s_mean;
		   s_std_m = s_std_m + region_val[r1.class_num].feat_res[i].s_std;
		   v_mean_m = v_mean_m + region_val[r1.class_num].feat_res[i].v_mean;
		   v_std_m = v_std_m + region_val[r1.class_num].feat_res[i].v_std;
		   colorfulness_m = colorfulness_m + region_val[r1.class_num].feat_res[i].colorfulness;
		   dom_color_m = dom_color_m + region_val[r1.class_num].feat_res[i].dom_color;
		   dom_color_2_m = dom_color_2_m + region_val[r1.class_num].feat_res[i].dom_color2;
	   }
	   s_mean_m = s_mean_m/region_val[r1.class_num].feat_res.size();
	   s_std_m = s_std_m/region_val[r1.class_num].feat_res.size();
	   v_mean_m = v_mean_m/region_val[r1.class_num].feat_res.size();
	   v_std_m = v_std_m/region_val[r1.class_num].feat_res.size();
	   colorfulness_m = colorfulness_m/region_val[r1.class_num].feat_res.size();
	   dom_color_m = dom_color_m/region_val[r1.class_num].feat_res.size();
	   dom_color_2_m = dom_color_2_m/region_val[r1.class_num].feat_res.size();

	   double s_mean = abs(s_mean_m - results2.s_mean);
	   double s_std = abs(s_std_m - results2.s_std);
	   double v_mean = abs(v_mean_m - results2.v_mean);
	   double v_std = abs(v_std_m - results2.v_std);
	   double colorfullness = abs(colorfulness_m - results2.colorfulness);
//	   double dom_color_1 = abs(dom_color_m - results2.dom_color);
//	   double dom_color_2 = abs(dom_color_2_m - results2.dom_color2);
//	   double dom_color_3 = abs(dom_color_m - results2.dom_color2);
//	   double dom_color_4 = abs(dom_color_2_m - results2.dom_color);

	   double color_diff = s_mean*1 + s_std*1 + v_mean*1 + v_std*1 + colorfullness*1;
//	   std::cout<<color_diff<<"color_diff"<<std::endl;

//	   if(dom_color_1 < 0.5 && dom_color_2 < 0.5 && results1.dom_color!=0 &&  results2.dom_color!=0)
//	   {
//		   color_diff = color_diff - 0.3;
//	   }else if(dom_color_1 < 0.5 && results1.dom_color!=0 &&  results2.dom_color!=0)
//	   {
//		   color_diff = color_diff -0.2;
//	   }else if((dom_color_3 < 0.5 || dom_color_4 < 0.5) && (results1.dom_color!=0 &&  results2.dom_color!=0) )
//	   {
//		   color_diff = color_diff -0.1;
//	   }else{
////		   color_diff=color_diff;
//	   }
//	   double img1_size = image1.rows * image1.cols;
//	   double img2_size = image2.rows * image2.cols;
//	   if(img1_size <=100)
//	   {
//		  color_diff = (color_diff*img1_size)/100;
//	   }
//	   if(img2_size <=100)
//	   {
//		  color_diff = (color_diff*img2_size)/100;
//	   }

//	   std::cout<<results1.s_mean <<"smean "<< results1.s_std <<"sstd "<< results1.v_mean <<"vmean "<< results1.v_std <<"vstd "<< results1.colorfulness <<"colorfull "<< results1.dom_color <<"dom1 "<< results1.dom_color2<<"dom2 "<<"Vec1"<< std::endl;
//	   std::cout<<results2.s_mean <<"smean "<< results2.s_std <<"sstd "<< results2.v_mean <<"vmean "<< results2.v_std <<"vstd "<< results2.colorfulness <<"colorfull "<< results2.dom_color <<"dom1 "<< results2.dom_color2<<"dom2 "<<"Vec2"<< std::endl;
//	   std::cout<<s_mean <<"smean "<< s_std <<"sstd "<< v_mean <<"vmean "<< v_std <<"vstd "<< colorfullness <<"colorfull "<< dom_color_1 <<"dom1 "<< dom_color_2<<"dom2 "<<color_diff<<"colordiff"<< std::endl;
//	   	double meandevr=0;
//		double meandevg=0;
//		double meandevb=0;
//	   		meandevr = meandevr + color_dev1.at<double>(0) +color_dev2.at<double>(0);
//	   		meandevg = meandevg + color_dev1.at<double>(1) +color_dev2.at<double>(1);
//	   		meandevb = meandevb + color_dev1.at<double>(2) +color_dev2.at<double>(2);
//	   	std::cout<<meandevr<<"r"<<std::endl;
//	   	std::cout<<meandevg<<"g"<<std::endl;
//	   	std::cout<<meandevb<<"b"<<std::endl;
//	if( merge_lbp_chi(r1, r2, region, image))//lbp && r2.validity && r1.lbp_set && r2.lbp_set &&)
//	if(meandevr < 30 && meandevg <30 && meandevb<30)
	if(color_diff <= mergefirstval)//2
	{
		r2.class_num = r1.class_num;
		region_val[r1.class_num].merged_pos = -1;
		region_val[r2.class_num].merged_pos = -1;
		region_val[r1.class_num].members.push_back(r2.id);
//		get_mean_color(r2, region[r1.class_num], image);
//		std::cout<<"lbpcheck"<<std::endl;
		return true;
	}else if(false)//check_hsv_two_region(r1, r2, image))
	{
		r2.class_num = r1.class_num;
		region_val[r1.class_num].members.push_back(r2.id);
//		get_mean_color(r2, region[r1.class_num], image);
//		std::cout<<"hsvcheck"<<std::endl;
		return true;
	}
	return false;
}
void merge_lbp(region& center, int class_num, std::vector<region_values>& region_class, cv::Mat &image) {

	int reg_num=0;
    if(center.childs.size()<1)
    {
    	if(center.validity)
    	{
			if(class_num==0)
			{
				reg_num = region_class.size();
				if(reg_num==0)reg_num=1;
				region_class.resize(reg_num+1);
				region_class[reg_num].members.push_back(center.id);

				center.class_num = reg_num;
				get_mean_color(center, region_class[reg_num], image);
			}else{
				reg_num =class_num;
			}


    		for(uint i=0;i<center.neighbors.size();i++)
			{
//    			int count=0;
//				int num = center.neighbors[i];
//				do ++count; while(num/=10);
//				std::vector<int> id;
//				for(int j = 0; j<count; j++)
//				{
//					if(j==0)id.push_back(center.neighbors[i]%10);
//					else id.push_back((((center.neighbors[i]%static_cast<int>((pow(10,j+1)))-center.neighbors[i]%static_cast<int>((pow(10,j))))/static_cast<int>(pow(10,j)))));
//				}
    			std::vector<int> id;
    			convert_id(center.neighbors[i], id);
				region *reg_ptr = get_region(id, r);
				region &reg_ref = *reg_ptr;
				if(group_region(center, reg_ref, region_class, image, center.validity) && reg_ref.validity)
				{
					center.validity = false;
//					reg_ref.validity=false;
					merge_lbp(reg_ref, reg_num, region_class, image);
				}
				center.validity = false;
    		}
		}
	}else
	{
		merge_lbp(center.childs[0], class_num, region_class, image);
		merge_lbp(center.childs[1], class_num, region_class, image);
		merge_lbp(center.childs[2], class_num, region_class, image);
		merge_lbp(center.childs[3], class_num, region_class, image);
	}
    return;
}
void merge_two_regions( std::vector<region_values>& region_class, int r1, int r2)
{
	for(uint i=0;i<region_class[r2].members.size();i++)
	{
		region_class[r1].members.push_back(region_class[r2].members[i]);
		region_class[r2].merged = true;
	}
}
void merge_lbp_second(region& center, int class_num, std::vector<region_values>& region_class, cv::Mat &image, double merge_number) {


	std::vector<region_values> region_merged;
	int size = region_class.size();
	std::vector< std::vector<int> > region_neighbor;
	region_neighbor.resize(size);
	std::vector<int> used_class;

	cv::Mat test = cv::Mat::zeros(525,700,CV_8UC3);
	//Get neighbor regions
	for(size_t a=0;a<region_class.size();a++)
	{
		for(size_t i=0;i<region_class[a].members.size();i++)
		{
			std::vector<int> id;
			convert_id(region_class[a].members[i], id);
			region *reg2 = get_region(id, r);
			region &reg = *reg2;
			region_class[a].merged_pos = -1;
			region_class[a].merged_second=true;
			reg.class_num=a;
//			std::cout<<reg.neighbors.size()<<"neighborsize"<<std::endl;
//			for(int x=reg.roi.x; x<reg.roi.x+reg.roi.width;x++)
//										{
//											for(int y=reg.roi.y;y<reg.roi.y+reg.roi.height;y++)
//											{
//												test.at<cv::Vec3b>(y,x)[0]=255;
//			//											test.at<cv::Vec3b>(y,x)[1]=0;
//			//											test.at<cv::Vec3b>(y,x)[2]=0;
//											}
//										}
//			for(int ib=0;ib<region_class[a].members.size();ib++)
////			{
//				std::vector<int> idn;
//								convert_id(region_class[a].members[ib], idn);
//								region *reg_n22 = get_region(idn, r);
//								region &reg2n = *reg_n22;
//								bool avoid_double = false;
////				//				std::cout<<regn.id<<"neighbor"<<j<<"   --"<<std::endl;
//								for(int x=reg2n.roi.x; x<reg2n.roi.x+reg2n.roi.width;x++)
//											{
//												for(int y=reg2n.roi.y;y<reg2n.roi.y+reg2n.roi.height;y++)
//												{
//													test.at<cv::Vec3b>(y,x)[0]=255;
//				//											test.at<cv::Vec3b>(y,x)[1]=0;
//				//											test.at<cv::Vec3b>(y,x)[2]=0;
//												}
//											}
//			}




			for(size_t j=0;j<reg.neighbors.size();j++)
			{
				std::vector<int> idn;
				convert_id(reg.neighbors[j], idn);
				region *reg_n2 = get_region(idn, r);
				region &regn = *reg_n2;
				bool avoid_double = false;
//				std::cout<<regn.id<<"neighbor"<<j<<"   --"<<std::endl;
//				for(int x=regn.roi.x; x<regn.roi.x+regn.roi.width;x++)
//							{
//								for(int y=regn.roi.y;y<regn.roi.y+regn.roi.height;y++)
//								{
//									test.at<cv::Vec3b>(y,x)[1]=255;
////											test.at<cv::Vec3b>(y,x)[1]=0;
////											test.at<cv::Vec3b>(y,x)[2]=0;
//								}
//							}

//				region_neighbor[a].push_back(regn.class_num);
//				std::cout<< reg.class_num <<"reg "<< regn.class_num << "reg2 "<<avoid_double<<"status"<<std::endl;
				if(regn.class_num != reg.class_num)
				{
					for(size_t ii=0; ii<region_neighbor[a].size();ii++)
					{
						if(region_neighbor[a][ii] == regn.class_num)
						{
							avoid_double = true;
						}

//						std::cout<< reg.class_num <<"reg "<< regn.class_num << "reg2 "<<avoid_double<<"status"<<std::endl;
					}
					if(!avoid_double || region_neighbor[a].size() == 0)
					{
						region_neighbor[a].push_back(regn.class_num);
//
//										for(int x=regn.roi.x; x<regn.roi.x+regn.roi.width;x++)
//													{
//														for(int y=regn.roi.y;y<regn.roi.y+regn.roi.height;y++)
//														{
//						//											test.at<cv::Vec3b>(y,x)[0]=255;
//															test.at<cv::Vec3b>(y,x)[1]=255;
//						//											test.at<cv::Vec3b>(y,x)[2]=0;
//														}
//													}
//										std::cout<<"Funzt<<<<<<<<<<<<<<<<"<<std::endl;
//					}else{
//
					}
				}else{
//
//
//					for(int x=regn.roi.x; x<regn.roi.x+regn.roi.width;x++)
//																		{
//																			for(int y=regn.roi.y;y<regn.roi.y+regn.roi.height;y++)
//																			{
//											//											test.at<cv::Vec3b>(y,x)[0]=255;
//																				test.at<cv::Vec3b>(y,x)[1]=255;
//											//											test.at<cv::Vec3b>(y,x)[2]=0;
//																			}
//																		}
				}


			}
//			cv::imshow("test", test);
//							cv::waitKey(10000);
//							test = cv::Mat::zeros(525,700,CV_8UC3);
		}
//		std::cout<<reg.neighbors.size()<<"num of neighbors"<<std::endl;

	}



	//Draw regions
	cv::Mat segment;
	std::vector <cv::Point> seg_points;
	std::vector< cv::Mat> segment_vec;
	for(size_t a=0;a<region_class.size();a++)
	{
		double sizeofregion=0;
		segment = cv::Mat::zeros(image.rows,image.cols, CV_8UC3);
		for(size_t i=0;i<region_class[a].members.size();i++)
		{
			std::vector<int> id;
			convert_id(region_class[a].members[i], id);
			region *reg2 = get_region(id, r);
			region &reg = *reg2;
			sizeofregion = sizeofregion + reg.roi.width*reg.roi.height;
			for(int x=reg.roi.x;x<reg.roi.width+reg.roi.x;x++)
			{
				for(int y=reg.roi.y;y<reg.roi.height+reg.roi.y;y++)
				{
					segment.at<cv::Vec3b>(y,x)[0]=image.at<cv::Vec3b>(y,x)[0];
					segment.at<cv::Vec3b>(y,x)[1]=image.at<cv::Vec3b>(y,x)[1];
					segment.at<cv::Vec3b>(y,x)[2]=image.at<cv::Vec3b>(y,x)[2];
					seg_points.push_back(cv::Point(x,y));
				}
			}
		}

//		used_class.push_back(a);
//		std::cout<<"begin of rotated rect"<<std::endl;
		if(sizeofregion>100)//40
		{
					cv::RotatedRect rec;
					if(seg_points.size()>3)
					{
						rec =  minAreaRect(seg_points);
					}
					seg_points.clear();
					cv::Mat M, rotated, new_segment;
					// get angle and size from the bounding box
					float angle = rec.angle;
					cv::Size rect_size = rec.size;
					if (rec.angle < -45.)
					{
						angle += 90.0;
						std::swap(rect_size.width, rect_size.height);
					}
					bool use_segment = false;
					M = cv::getRotationMatrix2D(rec.center, angle, 1.0);
					cv::warpAffine(segment, rotated, M, segment.size(), cv::INTER_CUBIC);
					cv::getRectSubPix(rotated, rect_size, rec.center, new_segment);
					use_segment=true;
					if(use_segment&& !new_segment.empty())
					{
						segment_vec.push_back(new_segment);
						used_class.push_back(a);
					}
		}
//		std::cout<<"end of rotated rect"<<std::endl;
		seg_points.clear();
	}
//	int count1 =0;
	//Get feature values of merged and optimized region image
	std::vector < std::vector<double> > hist_mean_feat;
	hist_mean_feat.resize(region_class.size());
//	int not_usable_num =0;
//	bool not_usable = false;

	for(size_t a=0;a<used_class.size();a++)
	{
//		bool not_usable = false;
		hist_mean_feat[a].resize(9);
		std::vector<struct feature_results> segment_features;
		struct feature_results results;
		texture_features edge = texture_features();
		cv::Mat dummy = cv::Mat::zeros(1,16,CV_32F);
		edge.primitive_size(&segment_vec[a], &results, &dummy);



//		if(		results.avg_size== results.avg_size &&
//				results.checked==results.checked &&
//				results.contrast==results.contrast &&
//				results.direct_reg==results.direct_reg &&
//				results.line_likeness==results.line_likeness &&
//				results.lined==results.lined &&
//				results.prim_num==results.prim_num &&
//				results.prim_regularity==results.prim_regularity &&
//				results.prim_strength==results.prim_strength){
////					std::cout<<results.avg_size<<" "<<results.checked<<" "<<results.contrast<<" "<<results.direct_reg<<" "<<results.line_likeness<<" "<< results.lined<<" "<< results.prim_num<<" "<< results.prim_regularity<<" "<< results.prim_strength<<"CORRECT"<<std::endl;
//				hist_mean_feat[a].push_back(results.avg_size);
//				hist_mean_feat[a].push_back(results.checked);
//				hist_mean_feat[a].push_back(results.contrast);
//				hist_mean_feat[a].push_back(results.direct_reg);
//				hist_mean_feat[a].push_back(results.line_likeness);
//				hist_mean_feat[a].push_back(results.lined);
//				hist_mean_feat[a].push_back(results.prim_num);
//				hist_mean_feat[a].push_back(results.prim_regularity);
//				hist_mean_feat[a].push_back(results.prim_strength);
//				std::cout<<"feature computation completed"<<std::endl;
////				std::cout<<results.avg_size<<" "<<results.checked<<" "<<results.contrast<<" "<<results.direct_reg<<" "<<results.line_likeness<<" "<< results.lined<<" "<< results.prim_num<<" "<< results.prim_regularity<<" "<< results.prim_strength<<"TRUE1"<<std::endl;
//		}else{
//				std::cout<<results.avg_size<<" "<<results.checked<<" "<<results.contrast<<" "<<results.direct_reg<<" "<<results.line_likeness<<" "<< results.lined<<" "<< results.prim_num<<" "<< results.prim_regularity<<" "<< results.prim_strength<<"FALSE2"<<std::endl;
//			used_class.erase(used_class.begin()+a);
//			std::cout<<"feature computation failed"<<std::endl;

			if(results.avg_size== results.avg_size)
			{
				hist_mean_feat[a].push_back(results.avg_size);
			}else{
				hist_mean_feat[a].push_back(0);
			}
			if(results.checked==results.checked)
			{
				hist_mean_feat[a].push_back(results.checked);
			}else{
				hist_mean_feat[a].push_back(0);
			}

			if(results.contrast==results.contrast)
			{
				hist_mean_feat[a].push_back(results.contrast);
			}else{
				hist_mean_feat[a].push_back(0);
			}

			if(results.direct_reg==results.direct_reg)
			{
				hist_mean_feat[a].push_back(results.direct_reg);
			}else{
				hist_mean_feat[a].push_back(0);
			}
			if(results.line_likeness==results.line_likeness)
			{
				hist_mean_feat[a].push_back(results.line_likeness);
			}else{
				hist_mean_feat[a].push_back(0);
			}

			if(results.lined==results.lined)
			{
				hist_mean_feat[a].push_back(results.lined);
			}else{
				hist_mean_feat[a].push_back(0);
			}

			if(results.prim_num==results.prim_num)
			{
				hist_mean_feat[a].push_back(results.prim_num);
			}else{
				hist_mean_feat[a].push_back(0);
			}


			if(results.prim_regularity==results.prim_regularity)
			{
				hist_mean_feat[a].push_back(results.prim_regularity);
			}else{
				hist_mean_feat[a].push_back(0);
			}

			if(results.prim_strength==results.prim_strength)
			{
				hist_mean_feat[a].push_back(results.prim_strength);
			}else{
				hist_mean_feat[a].push_back(0);
			}


//		}
	}
//	std::cout<<"end of features"<<std::endl;
	//Set feature vals of neighbors
//	std::cout<<used_class.size()<<"used_class"<<std::endl;
//	for(int a=0;a<used_class.size();a++)
//	{
		for(size_t a=0;a<used_class.size();a++)
		{
		if(region_class[used_class[a]].merged_second)
		{
//			std::cout<<"debug1"<<std::endl;
			double results=100000;
			std::vector < std::vector <double> > check_hist;
			double best_result_val = results;
			int best_result_pos = 0;
//			int used_position=0;
//			std::cout<<region_neighbor[a].size()<<"size of neighbors"<<std::endl;
			for(size_t j=0;j<region_neighbor[a].size();j++)
			{
				int position = 0;
				bool position_found = false;
				for(size_t i=0;i<used_class.size();i++)
				{
					if(used_class[i] == region_neighbor[a][j])
					{
						position = i;
						position_found = true;
					}
				}
//				position = used_class[position];//region_neighbor[a][j];
//				used_position = position_found;
				if(position_found && region_class[used_class[position]].merged_second)		//---------- überprüfung ob zu mergende region schon gemerge wurde angebracht? !region_class[region_neighbor[used_class[a]][j]].merged_second &&
				{
//					std::cout<<"debug2"<<std::endl;

					double cvval=0;

						for(size_t cv = 0; cv<hist_mean_feat[a].size();cv++)
						{
//							std::cout<<"debug21"<<std::endl;
//							std::cout<<hist_mean_feat[a][cv]<<" "<<hist_mean_feat[position][cv]<<std::endl;
							if(hist_mean_feat[a][cv]==hist_mean_feat[a][cv] && hist_mean_feat[position][cv]==hist_mean_feat[position][cv])
								cvval = cvval + abs(hist_mean_feat[a][cv]-hist_mean_feat[position][cv]);

//							std::cout<<"debug22"<<std::endl;
//							std::cout<<check_hist_short[0][cv]<<" -- "<<check_hist_short[1][cv]<<"values"<<std::endl;
						}
//						std::cout<<cvval<<"cvval"<<std::endl;

//						std::cout<<"debug3"<<std::endl;
						results=cvval;
//						std::cout<<results<<"results"<<std::endl;
						best_result_val=results;
						best_result_pos = used_class[position]; // region_neighbor[used_class[a]][j];

//						std::cout<<best_result_pos<<"best pos"<<std::endl;
//
//						std::cout<<"debug4"<<std::endl;
//						if(region_class[best_result_pos].merged_second)
//						{
				}}
						if(best_result_val <= mergesecondval)// 70 // && region_class[best_result_pos].merged_second)
						{


//							std::cout<<region_class[used_class[a]].merged_pos<<" "<< region_class[best_result_pos].merged_pos<<" "<<best_result_pos<<std::endl;
							if(region_class[used_class[a]].merged_pos == -1 && region_class[best_result_pos].merged_pos == -1)
							{
								region_merged.resize(region_merged.size()+1);
								for(size_t m = 0;m<region_class[best_result_pos].members.size();m++)
								{
									region_merged[region_merged.size()-1].members.push_back(region_class[best_result_pos].members[m]);
								}
								for(size_t m = 0;m<region_class[used_class[a]].members.size();m++)
								{
									region_merged[region_merged.size()-1].members.push_back(region_class[used_class[a]].members[m]);
								}
								region_class[best_result_pos].merged_second=false;
								region_class[used_class[a]].merged_second=false;
								region_class[used_class[a]].merged_pos=region_merged.size()-1;
								region_class[best_result_pos].merged_pos=region_merged.size()-1;
							}else if(region_class[used_class[a]].merged_pos != -1 && region_class[best_result_pos].merged_pos == -1)// && region_class[used_class[a]].merged_second==true && region_class[best_result_pos].merged_second==false)
							{
								for(size_t m = 0;m<region_class[best_result_pos].members.size();m++)
								{
									region_merged[region_class[used_class[a]].merged_pos].members.push_back(region_class[best_result_pos].members[m]);
								}
								region_class[best_result_pos].merged_pos=region_class[used_class[a]].merged_pos;
								region_class[best_result_pos].merged_second=false;
								region_class[used_class[a]].merged_second=false;
							}else if(region_class[used_class[a]].merged_pos == -1 && region_class[best_result_pos].merged_pos != -1)// && region_class[used_class[a]].merged_second==false && region_class[best_result_pos].merged_second==true)
							{
								for(size_t m = 0;m<region_class[used_class[a]].members.size();m++)
								{
									region_merged[region_class[best_result_pos].merged_pos].members.push_back(region_class[used_class[a]].members[m]);
								}
								region_class[used_class[a]].merged_pos=region_class[best_result_pos].merged_pos;
								region_class[used_class[a]].merged_second=false;
								region_class[best_result_pos].merged_second=false;
							}
							else if(region_class[used_class[a]].merged_pos!=-1 && region_class[best_result_pos].merged_pos!=-1 && region_class[a].merged_second==false && region_class[best_result_pos].merged_second==false)
							{
//								std::cout << "merge4"<<std::endl;
								int sizeregion=region_merged[region_class[best_result_pos].merged_pos].members.size();
								for(int m=0;m<sizeregion;m++)
								{
									region_merged[region_class[used_class[a]].merged_pos].members.push_back(region_merged[region_class[best_result_pos].merged_pos].members[m]);
								}
								for(size_t m=0;m<region_merged[region_class[best_result_pos].merged_pos].members.size();m++)
								{
									region_merged[region_class[best_result_pos].merged_pos].members.pop_back();
								}
								region_class[best_result_pos].merged_pos=region_class[used_class[a]].merged_pos;
							}else{
//								std::cout<<"failed!!"<<std::endl;
//								count1++;
//								region_merged.resize(region_merged.size()+1);
//								int size = region_class[a].members.size();
//								for(int m = 0;m<size;m++)
//								{
//									region_merged[region_merged.size()-1].members.push_back(region_class[used_class[a]].members.back());
//									region_class[a].members.pop_back();
//								}
//								std::cout<<"failed!! finished"<<std::endl;
							}

						}else{
//										region_merged.resize(region_merged.size()+1);
							//										for(int m = 0;m<region_class[used_class[a]].members.size();m++)
							//										{
							//											region_merged[region_merged.size()-1].members.push_back(region_class[used_class[a]].members[m]);
							//										}
//							region_class[best_result_pos].merged_second=false;
						}




		}
		if(region_class[used_class[a]].merged_second==true)
		{
//			region_merged.resize(region_merged.size()+1);
//			for(int m = 0;m<region_class[used_class[a]].members.size();m++)
//			{
//				region_merged[region_merged.size()-1].members.push_back(region_class[used_class[a]].members[m]);
//			}
//			region_class[used_class[a]].merged_second=false;
		}

	}

	for(size_t i=0;i<region_class.size();i++)
	{

				if(region_class[i].merged_second){
					region_merged.resize(region_merged.size()+1);
					for(size_t m = 0;m<region_class[i].members.size();m++)
					{
						region_merged[region_merged.size()-1].members.push_back(region_class[i].members[m]);
					}
				}
	}
//	std::cout<<"merging completed"<<std::endl;
//	std::cout<<region_merged.size()<<"mergesize"<<std::endl;
//	for(int i=0;i<region_merged.size();i++)
//	{
//		std::cout<<region_merged[i].members.size()<<"size of region"<< i <<std::endl;
//	}
//	std::cout<<region_class.size()<<"regionclass size   "<<region_merged.size()<<"regionmerge size"<<std::endl;
//	for(int i=0;i<region_merged.size();i++)
//	{
//		for(int j=0;j<region_merged[i].members.size();j++)
//		{
//			region_merged[i].merged_pos = -1;
//			region_merged[i].merged_second = false;
//		}
//	}
//
	if(region_merged.size()>0)
	{
		region_class.clear();
		region_class=region_merged;
//		std::cout<<"returend new class"<<std::endl;
	}
//	std::cout<<count1<<"anzahl der merges"<<std::endl;
//    return;
}
void set_region_lbp_mean(region reg, std::vector<region_values> &reg_val)
{


	int size = reg_val.size();
//	double lbp[10];
	std::vector<int> id;
	for(int i=0;i<size;i++)
	{
		int size_member = reg_val[i].members.size();
		for(int j=0; j<size_member;j++)
		{
			convert_id(reg_val[i].members[j], id);
//			region *reg2 = get_region(id, reg);

//			for(int k=0;k<10;k++)
//			{
//				reg_val[i].lbp[k] = reg_val[i].lbp[k] + (*reg2).lbp[k];
//			}
		}
//		for(int k=0;k<10;k++)
//		{
//			reg_val[i].lbp[k] = reg_val[i].lbp[k]/size_member;
//		}
	}
}
void set_region_rgb_mean(region reg, std::vector<region_values> &reg_val,cv::Mat image)
{


	int size = reg_val.size();
	std::vector<int> id;
	for(int i=0;i<size;i++)
	{
		int size_member = reg_val[i].members.size();
		for(int j=0; j<size_member;j++)
		{
			convert_id(reg_val[i].members[j], id);
			region *reg2 = get_region(id, reg);
			int x11 = (*reg2).roi.x;
			int x12 = (*reg2).roi.x + (*reg2).roi.width;
			int y11 = (*reg2).roi.y;
			int y12 = (*reg2).roi.y + (*reg2).roi.height;

			int r = 0;
			int g = 0;
			int b = 0;

			for(int j=y11;j<y12;j++)
			{
				for(int i=x11;i<x12;i++)
				{
					r = r + image.at<cv::Vec3b>(j,i)[2];
					g = g + image.at<cv::Vec3b>(j,i)[1];
					b = b + image.at<cv::Vec3b>(j,i)[0];
				}
			}
			r = r/((*reg2).roi.width*(*reg2).roi.height);
			g = g/((*reg2).roi.width*(*reg2).roi.height);
			b = b/((*reg2).roi.width*(*reg2).roi.height);

			reg_val[i].r = (reg_val[i].r+r);
			reg_val[i].g = (reg_val[i].g+g);
			reg_val[i].b = (reg_val[i].b+b);
		}
		if(size_member>0)
		{
		reg_val[i].r = (reg_val[i].r)/size_member;
		reg_val[i].g = (reg_val[i].g)/size_member;
		reg_val[i].b = (reg_val[i].b)/size_member;
		}
	}
}

void draw_region(cv::Mat img, region reg, std::vector<region_values> region) {

//	int size = region.size();
//	int color = 255/size;

	if(reg.childs.size()<1)
	{
    	for(int i = reg.roi.y; i<reg.roi.y+reg.roi.height;i++)
    	    	{
					for(int j = reg.roi.x; j<reg.roi.x+reg.roi.width;j++)
					{
							img.at<cv::Vec3b>(i,j)[2]=region[reg.class_num].r;
							img.at<cv::Vec3b>(i,j)[1]=region[reg.class_num].g;
							img.at<cv::Vec3b>(i,j)[0]=region[reg.class_num].b;
					}
    	    	}
	}else{

		for(uint i=0; i<reg.childs.size(); i++) {
			draw_region(img, reg.childs[i], region);
		}
	}
}
void draw_region_class(cv::Mat img, region reg, std::vector<region_values> region_class, std::vector<cv::Mat>* segments, int mergeval) {
	int size = region_class.size();

//	std::cout<<size<<"Anzahl der Klassenregionen"<<std::endl;

	cv::Mat test = cv::Mat::zeros(img.rows, img.cols, CV_32F);

	//	std::cout<<color<<"c1"<<color1<<"c2"<<color2<<"c3"<<std::endl;
	cv::Mat swap;

	for(int l=0;l<size;l++)
	{
		swap=cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
//		std::cout<<region_class[l].members.size()<<"region reg size"<<std::endl;
//		swap = cv::Mat::zeros(img.rows, img.cols, CV_8UC3);
		double region_surface=1;
		std::vector<cv::Point> segment_points;
		for(size_t mem=0;mem<region_class[l].members.size();mem++)
		{

			std::vector<int> id;
			convert_id(region_class[l].members[mem], id);
			region *reg_mem_ptr = get_region(id, r);
			region &reg_mem = *reg_mem_ptr;

				for(int i = reg_mem.roi.y; i<reg_mem.roi.y+reg_mem.roi.height;i++)
						{
							for(int j = reg_mem.roi.x; j<reg_mem.roi.x+reg_mem.roi.width;j++)
							{
			//						img.at<cv::Vec3b>(i,j)[2]=region_class[reg_mem.class_num].r;
			//						img.at<cv::Vec3b>(i,j)[1]=region_class[reg_mem.class_num].g;
			//						img.at<cv::Vec3b>(i,j)[0]=region_class[reg_mem.class_num].b;
									swap.at<cv::Vec3b>(i,j)[2]=img.at<cv::Vec3b>(i,j)[2];
									swap.at<cv::Vec3b>(i,j)[1]=img.at<cv::Vec3b>(i,j)[1];
									swap.at<cv::Vec3b>(i,j)[0]=img.at<cv::Vec3b>(i,j)[0];
//									test.at<cv::Vec3b>(i,j)[2]=img.at<cv::Vec3b>(i,j)[2];
//									test.at<cv::Vec3b>(i,j)[1]=img.at<cv::Vec3b>(i,j)[1];
//									test.at<cv::Vec3b>(i,j)[0]=img.at<cv::Vec3b>(i,j)[0];
									test.at<cv::Vec3b>(i,j) = 255;
									segment_points.push_back(cv::Point(j,i));
			//						img.at<cv::Vec3b>(i,j)[2]=color;
			//						img.at<cv::Vec3b>(i,j)[1]=color2;
			//						img.at<cv::Vec3b>(i,j)[0]=color1;
							}
						}
		}

//		cv::Mat test2 = swap.clone();
//		(*segments).push_back(test2);
		cv::RotatedRect rec;
		if(segment_points.size()>3)
		{
			rec =  minAreaRect(segment_points);
		}
		region_surface = region_surface + rec.size.width*rec.size.height;

		if(region_surface >=2000)
		{
			double fuellgrad = 0.25;
			if(mergeval == 2)
				fuellgrad = 0.35;
			cv::Mat binary_img;// = swap.clone();
//			binary_img.convertTo(binary_img,CV_8U,255.0/(255));
			cv::cvtColor( swap, binary_img, CV_BGR2GRAY );
			int non = countNonZero(binary_img);
			if(non/region_surface >=fuellgrad){
//				std::cout<<"insert image"<<std::endl;
				cv::Mat test2 = swap.clone();
				(*segments).push_back(test2);
			}
		}
	}
//	cv::imshow("test", test);
//	cv::waitKey(100000);
}
//void draw_region_class(cv::Mat img, region reg, std::vector<region_values> region_class) {
//
//	int size = region_class.size();
//
//
////	std::cout<<color<<"c1"<<color1<<"c2"<<color2<<"c3"<<std::endl;
//
//for(int l=0;l<size;l++)
//{
//	int color = rand() % 256;
//	int color1 = rand() % 256;
//	int color2 = rand() % 256;
//	for(int mem=0;mem<region_class[l].members.size();mem++)
//	{
//
//	std::vector<int> id;
//	convert_id(region_class[l].members[mem], id);
//	region *reg_mem_ptr = get_region(id, r);
//	region &reg_mem = *reg_mem_ptr;
//    	for(int i = reg_mem.roi.y; i<reg_mem.roi.y+reg_mem.roi.height;i++)
//    	    	{
//					for(int j = reg_mem.roi.x; j<reg_mem.roi.x+reg_mem.roi.width;j++)
//					{
//							img.at<cv::Vec3b>(i,j)[2]=region_class[reg_mem.class_num].r;
//							img.at<cv::Vec3b>(i,j)[1]=region_class[reg_mem.class_num].g;
//							img.at<cv::Vec3b>(i,j)[0]=region_class[reg_mem.class_num].b;
////						img.at<cv::Vec3b>(i,j)[2]=color;
////						img.at<cv::Vec3b>(i,j)[1]=color2;
////						img.at<cv::Vec3b>(i,j)[0]=color1;
//					}
//    	    	}
//	}
//	}
//}

void print_neighbor(region &t)
{
//	  clear_neighbor(t);
		if(t.childs.size()<1)
		{
		std::cout <<t.id << "id ";// << t.class_num<<"class num "<<t.lbp_set<<"lbpset";// << t.class_num << "class "<<std::endl;
		//std::cout <<t.childs.size() <<"size "<<t.neighbors.size()<<"nsize ";
//		for(int i = 0;i<t.neighbors.size();i++)
//		{
//			std::cout << t.neighbors[i]<< "neighbor"<<i<<" ";
//		}
		if(t.lbp_set)
		{
		for(int i = 0;i<10;i++)
		{
			std::cout << t.lbp[i]<<" ";
		}
		}
		std::cout <<std::endl;
		}
	if(t.childs.size()>1)
	{
		print_neighbor(t.childs[0]);
		print_neighbor(t.childs[1]);
		print_neighbor(t.childs[2]);
		print_neighbor(t.childs[3]);
	}
}
void get_num_lbp(region &t, int &lbp, int &notlbp)
{

//	if(t.lbp_set==true)lbp++;
//	else notlbp++;

	if(t.childs.size()>1)
	{
		notlbp++;
		get_num_lbp(t.childs[0], lbp, notlbp);
		get_num_lbp(t.childs[1], lbp, notlbp);
		get_num_lbp(t.childs[2], lbp, notlbp);
		get_num_lbp(t.childs[3], lbp, notlbp);
	}else lbp++;
}

void set_neighbor(std::vector<int> ids)
{
	for(size_t i=0; i<ids.size();i++)
	{
		std::vector<int> id;
		convert_id(ids[i], id);
		region *n1s = get_region(id, r);
		region &n1 = *n1s;
		for(size_t j=0;j<ids.size();j++)
		{
			if(j!=i)
			{
				std::vector<int> id2;
				convert_id(ids[j], id2);
				region *n2s = get_region(id2, r);
				region &n2 = *n2s;
				if(neighbor(n1, n2))
				{
					n1.neighbors.push_back(ids[j]);
				}
			}
		}
	}
}

void set_old_neighbors(region &n)
{
	if(n.childs.size()<1)
	{

		for(size_t i=0;i<n.N.size();i++){
			if(validity[n.N[i]])
				n.neighbors.push_back(n.N[i]);
//			std::cout<<n.N[i]<<" ";
		}

		for(size_t i=0;i<n.E.size();i++){
			if(validity[n.E[i]])
				n.neighbors.push_back(n.E[i]);
//			std::cout<<n.E[i]<<" ";
		}

		for(size_t i=0;i<n.S.size();i++){
			if(validity[n.S[i]])
				n.neighbors.push_back(n.S[i]);
//				std::cout<<n.S[i]<<" ";
		}

		for(size_t i=0;i<n.W.size();i++){
			if(validity[n.W[i]])
				n.neighbors.push_back(n.W[i]);
//				std::cout<<n.W[i]<<" ";
		}

//		std::cout<<n.neighbors.size()<<"Neighborsize "<<n.N.size()<<"N "<<n.E.size()<<"E "<<n.S.size()<<"S "<<n.W.size()<<"W "<<n.id<<std::endl;
	}else{
//		n.roi.height=0;
//		n.roi.width=0;
//		n.roi.x=0;
//		n.roi.y=0;


		set_old_neighbors(n.childs[0]);
		set_old_neighbors(n.childs[1]);
		set_old_neighbors(n.childs[2]);
		set_old_neighbors(n.childs[3]);
	}
}

splitandmerge::splitandmerge()
{
}
cv::Mat splitandmerge::categorize(cv::Mat image_in, std::vector<cv::Mat>* segments, double mergeval)
{
		original_image= image_in.clone();
		std::vector<region_values > region_class;
		std::vector<region_values > region_class_copy;
//		std::vector<cv::Mat>* segments_copy;
		double init[10];
	    cv::Mat img = image_in.clone();
	    std::vector<int> N, E, S, W;

	    validity.resize(4444444);

	    std::cout<<"split"<<std::endl;
//	    r = split(img, cv::Rect(0,0,img.cols,img.rows), init, 0, N, E, S, W, 0);
	    r = split(img, cv::Rect(0,0,img.cols,img.rows), init, 0);
//	    int a=0;
//	    int b=0;


	    timeval start, end;
	    gettimeofday(&start, NULL);

//	    get_num_lbp(r, a, b);
	    std::vector<int> ids;
//	    init_neighbour(r);
//	    std::cout<<"getneighbor"<<std::endl;
//	    get_neighbor(r, ids);
//	    std::cout<<"setneighbor"<<std::endl;
//	    set_neighbor(ids);



	    std::cout<<"setneighbor graph"<<std::endl;
	    neighbor_graph(r, N,E,S,W,0);
	    std::cout<<"set old neighbor"<<std::endl;
	    set_old_neighbors(r);


	    std::cout<<"clearneigbhor"<<std::endl;
	    clear_neighbor(r);


	    gettimeofday(&end, NULL);


	    double seconds = end.tv_sec - start.tv_sec;
	    double useconds = end.tv_usec - start.tv_usec;




	    std::cout<<seconds<<"used time "<<useconds<<"u used time"<<std::endl;

	    std::cout<<"merge first"<<std::endl;
//	    std::cout<<"neighbor_finished"<<std::endl;
	    merge_lbp(r, 0, region_class, img);



//	    for(int i=0;i<region_class.size();i++)
//	    {
//	    	for(int j=0; j<region_class[i].members.size();j++)
//	    	{
//	    		std::cout<<region_class[i].members[j]<<"  ";
//	    		std::vector<int> id;
//	    		convert_id(region_class[i].members[j], id);
//	    		region *reg2 = get_region(id, r);
//	    		region &reg = *reg2;
//	    		std::cout<<reg.class_num<<"---";
//	    		for(int k=0;k<reg.neighbors.size();k++)
//	    		{
//	    			std::cout<<reg.neighbors[k]<<" ";
//	    		}
//	    		std::cout<<"neighbors";
//
//	    	}
//	    	std::cout<<"NEW CLASS"<<std::endl;
//	    }

//	    std::cout<<"merged_firs_finished"<<std::endl;
	    std::cout<<"smergesecond"<<std::endl;
	    if(mergeval>0)
	    {
	    	merge_lbp_second(r, 0, region_class, img, 2);
//	    	std::cout<<"merged second"<<std::endl;
	    }
//	    std::cout<<"merged_second_finished"<<std::endl;
//	    std::cout<<region_class.size()<<"region_class1"<<std::endl;
//		draw_region_class(img, r, region_class, segments);
//		for(int i=0;i<(*segments).size();i++)
//		{
//			cv::imshow("seg1", (*segments)[i]);
//			cv::waitKey(10000);
//		}
//		(*segments).clear();
//	    merge_lbp_second(r, 0, region_class, img, 2);
//	    draw_region_class(img, r, region_class, segments);
//	    std::cout<<region_class.size()<<"region_class2"<<std::endl;
//		for(int i=0;i<(*segments).size();i++)
//		{
//			cv::imshow("seg2", (*segments)[i]);
//			cv::waitKey(10000);
//		}
//	    cv::waitKey(10000);

	    if(region_class.size()>=1)
	    {
			cv::Mat drawnregion = img.clone();
//			draw_rect(img, r);
			draw_region_class(img, r, region_class, segments, mergeval);
	    }
	    else
	    {
	    	std::cout<<"split image not"<<std::endl;
	    	(*segments).push_back(img);
	    }
//	    std::cout<<(*segments).size()<<"size of segments merged<<<<<<<<<<<<<<"<<std::endl;
	return img;
}
