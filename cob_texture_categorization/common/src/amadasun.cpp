#include "cob_texture_categorization/amadasun.h"
#include "cob_texture_categorization/texture_features.h"


//  MATLAB function to implement the 'Normalized NGTDM' (neighborhood
//  gray-tone difference matrix) method of texture analysis.
//
//  Method is as documented in "Texural Features Corresponding to
//  Texural Properties", M.Amadasun, R.King.
//  IEEE Transactions on Systems, Man, and Cybernetics, Vol 19, No 5,
//  Sep/Oct 1989.
//
//  Feature definitions normalized for images of differing size/shape
//  by dividing terms of sum(p*s) and sum(s) by R.
//
//  Inputs:
//    imagein	- input image
//    d          - neighborhood size
//
//  Outputs:
//    coars	- coarseness
//    contr	- contrast
//    busyn	- busyness
//    compl	- complexity
//    stren	- strength
//
//  Any pixel of value greater than 253 in the input image
//  is considered to be outside the region of interest, and is
//  excluded from the feature computation.


amadasun::amadasun()
{
}


void amadasun::get_amadasun(cv::Mat img,double d, struct feature_results *results, double& contrast_raw)
{

	std::cout << img.size() << std::endl;
	int type = img.type();
	//std::cout<<type<<"mattype";


	cv::Mat imagein1;
	cv::Mat imagein;
	cv::Mat imageinp1(img.rows, img.cols, CV_8UC1);
//	cvtColor(img, imagein, CV_BGR2GRAY);
	cvtColor(img, imagein1, CV_BGR2HSV);
	int from_to[] = { 2,0 };
	mixChannels( &imagein1, 1, &imageinp1, 1, from_to, 1 );
	imagein = imageinp1.clone();
	int greylevels=255;
	int rowsize = img.rows;
	int colsize = img.cols;

//	****************************************************************
//	Calculation of Neighborhood grey tone vector and normalization
//	coefficients.
//	****************************************************************

//	define neighborhood kernels

	cv::Mat oneskernel(2*d+1, 2*d+1, CV_8UC1);
	cv::Mat kernel(2*d+1, 2*d+1, CV_8UC1);
	for(int i=0;i<2*d+1;i++)
	{
		for(int j=0;j<2*d+1;j++)
		{
			oneskernel.at<uchar>(i,j)=1;
			kernel.at<uchar>(i,j)=1;
		}
	}
	kernel.at<uchar>(d,d)=0;
	double kerncount=(2*d+1)*(2*d+1)-1;

//	set ngtd vector and count vector to zero
	std::vector<double> s(greylevels);
	std::vector<double> n(greylevels);

//	increment entire image to give index into ngt and count vectors
	imageinp1 = imageinp1+1;

//	****************************************************************
//	select region of interest
//	****************************************************************

//	compute mask to select region of interest from image
//	mask=0 inside region, and +ve elsewhere
	int pix_val;
	cv::Mat mask = imagein.clone();
	for(int i=0;i<mask.rows;i++)
	{
		for(int j=0;j<mask.cols;j++)
		{
			pix_val = mask.at<uchar>(i,j);
			if(pix_val==254)
			{
				mask.at<uchar>(i,j)=1;
			}else if(pix_val==255)
			{
				mask.at<uchar>(i,j)=2;
			}else{
				mask.at<uchar>(i,j)=0;
			}
		}
	}

//	convolve with kernel of ones to select pixels for which the
//	kernel lies entirely within region of interest
//	convmask=1 where kernel fits into region, 0 otherwise
	cv::Mat convmask;
	cv::Point anchor(oneskernel.cols - oneskernel.cols/2 - 1, oneskernel.rows - oneskernel.rows/2 - 1);
	cv::flip(oneskernel, oneskernel, -1);
	cv::filter2D(mask, convmask, mask.depth(), oneskernel, anchor, 0, cv::BORDER_DEFAULT);

	for(int i=0;i<convmask.rows;i++)
	{
		for(int j=0;j<convmask.cols;j++)
		{
			int pix_val = convmask.at<uchar>(i,j);
			if(pix_val==0)
			{
				convmask.at<uchar>(i,j)=1;
			}else if(pix_val<0)
			{
				convmask.at<uchar>(i,j)=2;
			}else{
				convmask.at<uchar>(i,j)=0;
			}
		}
	}

//	****************************************************************
//	convolve kernel with image and compute NGTD matrix
//	****************************************************************
//
//	calculate neighbourhood average
	cv::Mat convimage(imagein.rows, imagein.cols, CV_32F);
	cv::Mat source;
	source = imagein.clone();
	cv::Point anchor2(kernel.cols - kernel.cols/2 - 1, kernel.rows - kernel.rows/2 - 1);
	cv::flip(kernel, kernel, -1);

	cv::filter2D(source, convimage, CV_32F, kernel, anchor2, 0, cv::BORDER_CONSTANT);
	convimage = convimage/kerncount;
//	calculate absolute differences between actual and
//	neighbourhood average grey levels

	for(int i=0;i<convimage.rows;i++)
	{
		for(int j=0;j<convimage.cols;j++)
		{
			float pix_val = convimage.at<float>(i,j);
			float img_val = imagein.at<uchar>(i,j);
			float conv_val = img_val-pix_val;
			if(conv_val<0)
			{
				conv_val = conv_val*(-1);
				convimage.at<float>(i,j)=conv_val;
			}else
			{
				convimage.at<float>(i,j)=conv_val;
			}
		}
	}

//	work through convolved image constructing NGTD matrix
	for(int i=d;i<=rowsize-d;i++)
	{
		for(int j=d;j<=colsize-d;j++)
		{
			int conv_val = convmask.at<uchar>(i,j);
			if(conv_val>0)
			{
				int index=imageinp1.at<uchar>(i,j);
				float conv_add = convimage.at<float>(i,j);
				s[index]=s[index] + conv_add;
				n[index]=n[index]+1;
			}
		}
	}

//	calculate normalization coefficient
	double r=0;
	for(uint i=0;i<n.size();i++)
	{
		r = r+n[i];
	}

//	*******************************************************************
//	Calculate features
//	*******************************************************************
//
//	calculate useful matrices

	double size = n.size();
	cv::Mat ni(size, size, CV_32FC1);
	cv::Mat nj(size, size, CV_32FC1);
	cv::Mat si(size, size, CV_32FC1);
	cv::Mat sj(size, size, CV_32FC1);
	cv::Mat i_mat(greylevels, greylevels, CV_32FC1);
	cv::Mat j_mat(greylevels, greylevels, CV_32FC1);

	for (int i = 1; i < size; i++)
	{
		ni.at<float>(0,(i-1)) = n[i];
		nj.at<float>(i-1,0) = n[i];
		si.at<float>(0,i) = s[i];
		sj.at<float>(i,0) = s[i];
	}
    ni = cv::repeat(ni.row(0),size,1);
    nj = cv::repeat(nj.col(0),1,size);
    si = cv::repeat(si.row(0),size,1);
    sj = cv::repeat(sj.col(0),1,size);
	for (int i = 1; i <= greylevels-1; i++)
	{
		i_mat.at<float>(0,i) = i;
		j_mat.at<float>(i,0) = i;
	}
    i_mat = cv::repeat(i_mat.row(0),size,1);
    j_mat = cv::repeat(j_mat.col(0),1,size);
    cv::Mat ilessjsq(greylevels, greylevels, CV_32FC1);

	for(int i=0;i<ilessjsq.rows;i++)
	{
		for(int j=0;j<ilessjsq.cols;j++)
		{
			ilessjsq.at<float>(i,j)= (i_mat.at<float>(i,j)-j_mat.at<float>(i,j))*(i_mat.at<float>(i,j)-j_mat.at<float>(i,j));
		}
	}

//  Elements of Ni and Nj are required to be zero where either
//  Ni or Nj are zero.
    float nij_sum=0;
	for(int i=0;i<ni.rows-1;i++)
	{
		for(int j=0;j<ni.cols-1;j++)
		{
			float ni_val = ni.at<float>(i,j);
			float nj_val = nj.at<float>(i,j);
			float il_val = ilessjsq.at<float>(i,j);
			if(ni_val==0) nj.at<float>(i,j)=0;
			if(nj_val==0) ni.at<float>(i,j)=0;
			nij_sum = nij_sum + ni_val*nj_val*il_val;
		}
	}

//	coarseness, busyness and strenght are not necessary for feature detection

//	coarseness
//    double coars;
    int ng=0; //var of contrast
    int sum_s=0;
//    double ns_sum;
	for(uint i=0;i<n.size();i++)
	{
			float n_val = n[i];
			float s_val = s[i];
			sum_s = sum_s + s_val;
			if(n_val!=0)ng=ng+1;
//			ns_sum = ns_sum + n_val*s_val;
	}
//	coars =(r*r)/ns_sum;

//	contrast -- Value 12
	double contr = sum_s*nij_sum/(r*r*r)/ng/(ng-1);
	contrast_raw = contr;
	contr = 1.7*pow(contr,3)-4.5*pow(contr, 2)+6.9*contr+1.4;
	//std::cout<<contr<<"contrast_vorresult"<<std::endl;
	if(contr<1)contr=1;
	if(contr>5)contr=5;

//	busyness
//	assumes that absolute value of sum of differences between
//	weighted grey scale valuesis intended, in accordance with
//	textual description in source paper
//	float busyn_sum=0;
//	float busyn=0;
//	for(int i=0;i<ni.rows-1;i++)
//	{
//		for(int j=0;j<ni.cols-1;j++)
//		{
//			float ni_val = ni.at<float>(i,j);
//			float nj_val = nj.at<float>(i,j);
//			float i_val = i_mat.at<float>(i,j);
//			float j_val = j_mat.at<float>(i,j);
//			float busyn_val = i_val*ni_val-j_val*nj_val;
//			if(busyn_val<0) busyn_val = busyn_val*(-1);
//			busyn_sum = busyn_sum + busyn_val;
//		}
//	}
//	busyn = ns_sum/busyn_sum/r;
//	complexity
//	float busyn_val;
//	float eps = pow(2, -52);
//	float eps_sum;
//	float compl_val=0;
//	for(int i=0;i<ni.rows-1;i++)
//	{
//		for(int j=0;j<ni.cols-1;j++)
//		{
//			float ni_val = ni.at<float>(i,j);
//			float nj_val = nj.at<float>(i,j);
//			float si_val = si.at<float>(i,j);
//			float sj_val = sj.at<float>(i,j);
//			float i_val = i_mat.at<float>(i,j);
//			float j_val = j_mat.at<float>(i,j);
//			float ij_abs = i_val-j_val;
//			if(ij_abs<0)ij_abs=ij_abs*(-1);
//			eps_sum = eps+ni_val+nj_val;
//			busyn_val = ((si_val*ni_val+sj_val*nj_val)*ij_abs)/eps_sum;
//			compl_val = compl_val + busyn_val;
//		}
//	}
//	compl_val = compl_val/r;
//
//	strength
//	float stren;
//	for(int i=0;i<ni.rows;i++)
//	{
//		for(int j=0;j<ni.cols;j++)
//		{
//			int ni_val = ni.at<float>(i,j);
//			int nj_val = nj.at<float>(i,j);
//			stren = stren + (ni_val + nj_val)*ilessjsq.at<float>(i,j);
//		}
//	}
//	stren = stren /(sum_s+eps);
//	(*results).coars = coars;
	(*results).contrast = contr;
//	(*results).busyn = busyn;
//	(*results).compl_val = compl_val;
//	(*results).stren = stren;


}
