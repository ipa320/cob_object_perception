#include "cob_texture_categorization/segment_trans.h"


segment_trans::segment_trans()
{
}

void segment_trans::transformation(cv::Mat *source, cv::Mat *dest, cv::Mat *depth)
{
//	Steigung x-Richtung
	float ysum=0;
	float xsum=0;
	float y1 = (*depth).at<float>(200,319);
	float y2 = (*depth).at<float>(280,319);
	int ydown = 200;
	int yup = 280;

	while(y1==0 && y2!=0 && ydown>=0)
	{
		ydown--;
		y1 = (*depth).at<float>(ydown,319);
	}
	while(y1!=0 && y2==0 && yup<640)
	{
		yup++;
		y2 = (*depth).at<float>(yup,319);
	}

	float x1 = (*depth).at<float>(239,280);
	float x2 = (*depth).at<float>(239,360);
	int xdown = 280;
	int xup = 360;

	while(x1==0 && x2!=0 && xdown >=0)
	{
		xdown--;
		x1 = (*depth).at<float>(239,xdown);
	}
	while(x1!=0 && x2==0 && xup<640)
	{
		xup++;
		x2 = (*depth).at<float>(239,xup);
	}

	float my = (y1-y2)/80;
	float mx = (x2-x1)/80;
	for(int i = 280; i<361;i++)
	{
		(*source).at<cv::Vec3b>(239,i)[2]=255;
		(*source).at<cv::Vec3b>(239,i)[1]=0;
		(*source).at<cv::Vec3b>(239,i)[0]=0;
		xsum=xsum+(*depth).at<float>(239,i)*106;


	}
	for(int i = 200; i<281;i++)
	{
		(*source).at<cv::Vec3b>(i,319)[2]=255;
		(*source).at<cv::Vec3b>(i,319)[1]=0;
		(*source).at<cv::Vec3b>(i,319)[0]=0;
		ysum=ysum+(*depth).at<float>(i,319)*80;
	}

//	double my = ysum/(80*80);
//	double mx = xsum/(80*80);
	std::cout<< 100*mx/std::max(x1,x2) << "mx   "<< 100*my/std::max(y1,y2)<<"my  :"<<std::endl;
//	std::cout << std::max(x1,x2)<<"max "<<x1<<"x1 "<<x2<<"x2 ";
//
	if(mx <1.7 && mx >(-1.7))
	{
		mx=mx*275;
	}else if(mx<=0.25&&mx>(-0.25))
	{
		mx=mx*138;
	}else{
		mx=0;
	}
	if(my<=0.25&& my>(-0.25))
	{
		my=my*180;
	}else if(mx<1.7 && mx>(-1.7))
	{
		my=my*138;
	}else{
		my=0;
	}
	std::cout<< mx << "mx   "<< my<<"my  :"<<std::endl;


//	Rotation um x-Achse
//	x' := x
//	  	y' := y · cos($\alpha$) - z · sin($\alpha$)
//	  	z' := y · sin($\alpha$) + z · cos($\alpha$)
	cv::Mat xrot(480, 640, CV_8UC3);


//	for(int i=0; i<(*source).rows;i++)
//	{
//		for(int j=0; j<(*source).cols;j++)
//		{
//			int yn= round(j*cos(my));
//			std::cout<< yn <<"yn "<<i<<"i "<<j<<"j "<<std::endl;
////			if(yn>=0 && yn<480)
////			{
////			xrot.at<cv::Vec3b>(yn,i)[0]=(*source).at<cv::Vec3b>(j,i)[0];
////			xrot.at<cv::Vec3b>(yn,i)[1]=(*source).at<cv::Vec3b>(j,i)[1];
////			xrot.at<cv::Vec3b>(yn,i)[2]=(*source).at<cv::Vec3b>(j,i)[2];
////			}else
////			{
////				std::cout <<yn<< "xn ueberschreitung"<<std::endl;
////			}
//		}
//	}
////	Rotation um y -Achse 	x' := z · sin($\alpha$) + x · cos($\alpha$)
////	  	y' := y
////	  	z' := z · cos($\alpha$) - x · sin($\alpha$)
//	for(int i=0; i<(*source).rows;i++)
//	{
//		for(int j=0; j<(*source).cols;j++)
//		{
//			int xn= round(i*cos(mx));
//			std::cout<< xn <<"xn "<<i<<"i "<<j<<"j "<<std::endl;
////			if(xn>=0 && xn<640)
////			{
////			(*dest).at<cv::Vec3b>(j,xn)[0]=xrot.at<cv::Vec3b>(j,i)[0];
////			(*dest).at<cv::Vec3b>(j,xn)[1]=xrot.at<cv::Vec3b>(j,i)[1];
////			(*dest).at<cv::Vec3b>(j,xn)[2]=xrot.at<cv::Vec3b>(j,i)[2];
////			}else
////			{
////				std::cout << "xn ueberschreitung"<<std::endl;
////			}
//		}
//	}




//	for(int i=0; i<(*source).rows;i++)
//	{
//		for(int j=0; i<(*source).cols;j++)
//		{
//
//		}
//	}



}
