#include "object_categorization/OpenCVUtils.h"

namespace ipa_utils {

void DblMatrixList::PushBackIplImage(IplImage* Image)
{
	assert(Image->nChannels==1);
	DblMatrix m;
	m.Assign(Image->width, Image->height, 0.0);
	for(int j=0; j<Image->height; j++)
		for(int i=0; i<Image->width; i++)
			m[j][i]=cvGetReal2D(Image, j, i);
	push_back(m);
}

void DblMatrixList::ImportFromIplImageXML(std::string Name, bool Append)
{
	if(!Append) clear();
	IplImage* Image;
	char RealName[256];
	for(int i=0; true; i++)
	{
		sprintf(RealName, Name.c_str(), i);
		Image = (IplImage*)cvLoad(RealName);
		if(Image==NULL) break;
		else PushBackIplImage(Image);
	}
	cvReleaseImage(&Image);
}

unsigned long DblMatrixList::Load(std::string PathName, std::string FileName, std::string InfoFileName)
{
	clear();
	
	/// load info header
	std::stringstream FileNameStream;
	FileNameStream << PathName << InfoFileName;
	int s=0;
	std::ifstream f(FileNameStream.str().c_str());
	if(!f.is_open()) return RET_FAILED;
	f >> s;
	f.close();

	for(int i=0; i<s; i++)
	{
		DblMatrix Temp;
		std::stringstream Num;
		Num << PathName << FileName << i << ".txt";
		if(Temp.Load(Num.str()) & RET_FAILED) return RET_FAILED;
		push_back(Temp);
	}
	return RET_OK;
}

unsigned long DblMatrixList::Delete(std::string PathName, std::string FileName, std::string InfoFileName)
{
	/// load old info header
	std::stringstream FileNameStream;
	FileNameStream << PathName << InfoFileName;
	int s=0;
	std::ifstream f(FileNameStream.str().c_str());
	if(f.is_open())	//delete old sequence
	{
		f >> s;
		for (int i=0; i<s; i++)
		{
			std::stringstream FileNameStream2;
			FileNameStream2 << PathName << FileName << i << ".txt";
			remove((FileNameStream2.str()).c_str());
		}
		f.close();
		remove((FileNameStream.str()).c_str());
	}
	return RET_OK;
}

unsigned long DblMatrixList::Save(std::string Name)
{
	for(int i=0; i<(int)size(); i++)
	{
		std::stringstream Num;
		Num << Name << i << ".txt";
		if((*this)[i].Save(Num.str())==RET_FAILED) return RET_FAILED;
	}
	return RET_OK;
}

void DblMatrixList::GetLine(int k, int j, DblVector& Line, int Step)
{
	(*this)[k].GetLine(j, Line, Step);
}

void DblMatrixList::GetColumn(int k, int i, DblVector& Column, int Step)
{
	(*this)[k].GetColumn(i, Column, Step);
}

void ConvertToShowImage(IplImage* Source, IplImage* Dest)
{
        double Min, Max;
        cvMinMaxLoc(Source, &Min, &Max);
		double w = Max-Min;
        for(int j=0; j<Source->height; j++)
		{
			for(int i=0; i<Source->width; i++)
			{
				double d = cvGetReal2D(Source, j, i);
				int V= (int)(255.0 * ((d-Min)/w));
	
				CvScalar Color = CV_RGB(V, V, V);
				cvSet2D(Dest, j, i, Color);
			}
		}
}

void GetMinMax(IplImage* Image, Point3Dbl& Min, Point3Dbl& Max)
{
	Min.m_x = DBL_MAX;
	Min.m_y = DBL_MAX;
	Min.m_z = DBL_MAX;

	Max.m_x = -DBL_MAX;
	Max.m_y = -DBL_MAX;
	Max.m_z = -DBL_MAX;

	for(int j=0; j<Image->height; j++)
	{
		for(int i=0; i<Image->width; i++)
		{
			CvScalar Val = cvGet2D(Image, j, i);
			if((float)Val.val[0]<Min.m_x) Min.m_x = (float)Val.val[0];
			if((float)Val.val[1]<Min.m_y) Min.m_y = (float)Val.val[1];
			if((float)Val.val[2]<Min.m_z) Min.m_z = (float)Val.val[2];
			if((float)Val.val[0]>Max.m_x) Max.m_x = (float)Val.val[0];
			if((float)Val.val[1]>Max.m_y) Max.m_y = (float)Val.val[1];
			if((float)Val.val[2]>Max.m_z) Max.m_z = (float)Val.val[2];
		}
	}
}

double BoxSum(IplImage* SrcInt, int i, int j, int r)
{	
	double br = cvGetReal2D(SrcInt, j+r+1, i+r+1);
	double bl = cvGetReal2D(SrcInt, j+r+1, i-r);
	double tr = cvGetReal2D(SrcInt, j-r, i+r+1);
	double tl = cvGetReal2D(SrcInt, j-r, i-r);
	
	// todo: make faster
	//double BR = ((unsigned int*)(SrcInt->imageData + (j+r+1) * SrcInt->widthStep))[(i+r+1) * SrcInt->nChannels];
	//double BL = ((unsigned int*)(SrcInt->imageData + (j+r+1) * SrcInt->widthStep))[(i-r) * SrcInt->nChannels];
	//double TR = ((unsigned int*)(SrcInt->imageData + (j-r) * SrcInt->widthStep))[(i+r+1) * SrcInt->nChannels];
	//double TL = ((unsigned int*)(SrcInt->imageData + (j-r) * SrcInt->widthStep))[(i-r) * SrcInt->nChannels];

	return br-bl-tr+tl;
}

CvScalar BoxSumCvScalar(IplImage* SrcInt, int i, int j, int r)
{
	CvScalar v0 = cvGet2D(SrcInt, j+r+1, i+r+1);
	CvScalar v1 = cvGet2D(SrcInt, j-r, i+r+1);
	CvScalar v2 = cvGet2D(SrcInt, j+r+1, i-r);
	CvScalar v3 = cvGet2D(SrcInt, j-r, i-r);
	return cvScalar(v0.val[0]-v1.val[0]-v2.val[0]+v3.val[0],
					v0.val[1]-v1.val[1]-v2.val[1]+v3.val[1],
					v0.val[2]-v1.val[2]-v2.val[2]+v3.val[2]);												
}

void RangeDisplayImageConversion(IplImage* Range, IplImage* Out, double* Min, double* Max)
{	
	double MinR=5000.0, MaxR=30000.0; // these values are for the raw data
	if(Min!=NULL) MinR=*Min;
	if(Max!=NULL) MaxR=*Max;
	double wr=MaxR-MinR;
	
	for(int j=0; j<Range->height; j++)
	{
		for(int i=0; i<Range->width; i++)
		{
			double d = cvGetReal2D(Range, j, i);
			if(d>MaxR) d=MaxR;
			if(d<MinR) d=MinR;
			double H = 180.0 * (d-MinR)/wr;			
			double S = 255.0;
			double V = 127.0;
			//if(d>MaxR || d<MinR) V=0;
			if(d>=MaxR) V=0;
			CvScalar Color = CV_RGB(V, S, H);
			cvSet2D(Out, j, i, Color);
		}
	}
	cvCvtColor(Out, Out, CV_HSV2RGB);
}

void IntenDisplayImageConversion(IplImage* Intensity, IplImage* Out)
{
	cvConvertScaleAbs(Intensity, Out, 1.0/64.0);
	//cvEqualizeHist(Out, Out);
	
	//double MinI = 0;//250;
	//double MaxI = 5000;
	//cvMinMaxLoc(Intensity, &MinI, &MaxI);
	//double wi=MaxI-MinI;
	//for(int j=0; j<Intensity->height; j++)
	//{
	//	for(int i=0; i<Intensity->width; i++)
	//	{	
	//		double I = cvGetReal2D(Intensity, j, i);
	//		if(I>MaxI) I=MaxI;
	//		if(I<MinI) I=MinI;
	//		double V = 255.0 * (I-MinI)/wi;
	//		CvScalar Color = CV_RGB(V, V, V);
	//		cvSet2D(Out, j, i, Color);
	//	}
	//}
}


void RangeDisplayImageConversion(IplImage* Range, IplImage* Intensity, IplImage* Out)
{
	double MinR=2000.0, MaxR=60000.0;
	double MinI = 1000;
	double MaxI = 15000;

	double wr=MaxR-MinR;
	double wi=MaxI-MinI;
	
	for(int j=0; j<Range->height; j++)
	{
		for(int i=0; i<Range->width; i++)
		{
			double H = 180.0 * (dblmin(cvGetReal2D(Range, j, i), MaxR)-MinR)/wr;			
			double S = 255.0;
			double V = 255.0 * (dblmin(cvGetReal2D(Intensity, j, i), MaxI)-MinI)/wi;
			CvScalar Color = CV_RGB(V, S, H);
			cvSet2D(Out, j, i, Color);
		}
	}
	cvCvtColor(Out, Out, CV_HSV2RGB);
}

void CoordDisplayImageConversion(IplImage* CoordImg, IplImage* Out,
								 Point3Dbl* Min, Point3Dbl* Max)
{	
	Point3Dbl min;
	Point3Dbl max;
	if(Min==NULL && Max==NULL)
		GetMinMax(CoordImg, min, max);
	else
	{
		min = *Min;
		max = *Max;
	}

	double wx = max.m_x-min.m_x;
	double wy = max.m_y-min.m_y;
	double wz = max.m_z-min.m_z;
	IplImage* Temp = cvCreateImage(cvGetSize(CoordImg), IPL_DEPTH_8U, 3);
	for(int j=0; j<CoordImg->height; j++)
	{
		for(int i=0; i<CoordImg->width; i++)
		{
			CvScalar Value = cvGet2D(CoordImg, j, i);
			double X = floor(255.0 * ((double)Value.val[0]-min.m_x)/wx);
			double Y = floor(255.0 * ((double)Value.val[1]-min.m_y)/wy);
			double Z = floor(255.0 * ((double)Value.val[2]-min.m_z)/wz);
			cvSet2D(Temp, j, i, cvScalar((int)X, (int)Y, (int)Z));
		}
	}
	cvResize(Temp, Out);
	cvReleaseImage(&Temp);
}

void CoordDisplayImageConversionOnlyZSpectral(IplImage* CoordImg, IplImage* Out, double MinZ, double MaxZ)
{
	double wr=MaxZ-MinZ;	
	for(int j=0; j<CoordImg->height; j++)
	{
		for(int i=0; i<CoordImg->width; i++)
		{
			CvScalar Val = cvGet2D(CoordImg, j, i);
			double z = Val.val[2];
			if(z>MaxZ) z=MaxZ;
			if(z<MinZ) z=MinZ;
			double H = 360 * (z-MinZ)/wr;			
			double S = 255.0;
			double V = 127.0;
			if(z>=MaxZ) V=0;
			CvScalar Color = CV_RGB(V, S, H);
			cvSet2D(Out, j, i, Color);
		}
	}
	cvCvtColor(Out, Out, CV_HSV2RGB);
}

void CoordDisplayImageConversionOnlyZGray(IplImage* CoordImg, IplImage* Out, double MinZ, double MaxZ)
{
	double wr=MaxZ-MinZ;	
	for(int j=0; j<CoordImg->height; j++)
	{
		for(int i=0; i<CoordImg->width; i++)
		{
			CvScalar Val = cvGet2D(CoordImg, j, i);
			double z = Val.val[2];
			if(z>MaxZ) z=MaxZ;
			if(z<MinZ) z=MinZ;
			int Gray = cvRound(255.0 * (z-MinZ)/wr);			
			CvScalar Color = CV_RGB(Gray, Gray, Gray);
			cvSet2D(Out, j, i, Color);
		}
	}
}

void SaveImageAsMatrix(std::string FileName, IplImage* DblImage)
{
	DblMatrix Matrix;
	for(int j=0; j<DblImage->height; j++)
	{
		DblVector Tmp;
		Matrix.push_back(Tmp);
		for(int i=0; i<DblImage->width; i++)
		{
			double Val = cvGetReal2D(DblImage, j, i);
			Matrix[j].push_back(Val);
		}
	}
	Matrix.Save(FileName);
}

void LoadImageAsMatrix(std::string FileName, IplImage* DblImage)
{
	DblMatrix Matrix;
	for(int j=0; j<DblImage->height; j++)
	{
		Matrix.push_back(DblVector());
		for(int i=0; i<DblImage->width; i++)
		{
			double Val = cvGetReal2D(DblImage, j, i);
			Matrix[j].push_back(Val);
		}
	}
	Matrix.Save(FileName);
}

void ShowPointsInImage(IplImage* Img, const std::vector<CvPoint>& LimbPoints)
{
	for(unsigned int j=0; j<LimbPoints.size(); j++)
		cvCircle(Img, LimbPoints[j], 1, CV_RGB(0, 0, 255), 2);			
}

void MinFilter(IplImage* Image)
{
	assert(Image->nChannels==1);
	IplImage* Temp = cvCloneImage(Image);
	int j, k;
	for(j=1; j<Image->height-1; j++)
		for(k=1; k<Image->width-1; k++)
		{
			double c = cvGetReal2D(Temp, j, k);
			double l = cvGetReal2D(Temp, j, k-1);
			double r = cvGetReal2D(Temp, j, k+1);
			double t = cvGetReal2D(Temp, j-1, k);
			double b = cvGetReal2D(Temp, j+1, k);
			c = dblmin(c, dblmin(dblmin(l, r), dblmin(t, b)));
			cvSetReal2D(Image, j, k, c);
		}
	cvReleaseImage(&Temp);
}


/// Copies a rotated image into another image
int CopyRotatedImageFrame(IplImage* Input, IplImage* Output, CvPoint Center, double Angle, int Fill, double sx, double sy)
{
	double SinRot = sin(Angle);
	double CosRot = cos(Angle);
	int xmin = -(int)floor((double)Output->width/2.0);
	int xmax = -xmin +  Output->width%2;
	int ymin = -(int)floor((double)Output->height/2.0);
	int ymax = -ymin +  Output->height%2;
	for(int xi = xmin; xi < xmax; xi++)
	{
		for(int yi = ymin; yi < ymax; yi++)
		{  
			// iterate over all pixels in the frame	
			double xpos = Center.x + sx + CosRot*xi - SinRot*yi; // x position on the source image
			double ypos = Center.y + sy + CosRot*yi + SinRot*xi; // y position on the source image
			
			if(Input->nChannels==1)
			{
				if(xpos<0 || xpos>=Input->width || ypos<0 || ypos>=Input->height)
					cvSetReal2D(Output, yi+Output->height/2, xi+Output->width/2, Fill);
				else
					cvSetReal2D(Output, yi+Output->height/2, xi+Output->width/2, InterpolRealValue(Input, xpos, ypos));
			}
			else if(Input->nChannels==3)
			{
				if(xpos<0 || xpos>=Input->width || ypos<0 || ypos>=Input->height)
					cvSet2D(Output, yi+Output->height/2, xi+Output->width/2, CV_RGB(Fill, Fill, Fill));
				else
					cvSet2D(Output, yi+Output->height/2, xi+Output->width/2, InterpolValue(Input, xpos, ypos));
			}
			else return -1;
		}
	}
	return 0;
}

double InterpolRealValue(IplImage* Img, double xpos, double ypos){

	int x1 = (int)floor(xpos); // Pixelspalte links von dem Punkte
	int x2 = (int)ceil(xpos); // Pixelspalte rechts von dem Punkte
	int y1 = (int)floor(ypos); // Pixelreihe �ber dem Punkte
	int y2 = (int)ceil(ypos); // Pixelreihe �ber dem Punkte
	if(x2>=Img->width-1 || y2>=Img->width-1) return 0.0;
	double xInfl = (double)xpos - x1; // Einflu� der linken Spalte
	double yInfl = (double)ypos - y1; // Einflu� der oberen Spalte
	// abfrage 
	double uRVal =  				// Interpolation in der oberen Pixelreihe
		(double)cvGetReal2D(Img, y1, x1) * (1-xInfl) +
		(double)cvGetReal2D(Img, y1, x2) * (xInfl); 
				
	double lRVal = 				// Interpolation in der unteren Pixelreihe	 
		(double)cvGetReal2D(Img, y2, x1) * (1-xInfl) +
		(double)cvGetReal2D(Img, y2, x2) * (xInfl); 
		
	double retVal = uRVal * (1.0-yInfl) + lRVal * yInfl; 
								// zwischen den Reihen interpolieren
	return retVal;
}

CvScalar InterpolValue(IplImage* Img, double xpos, double ypos)
{

	int x1 = (int)floor(xpos); // Pixelspalte links von dem Punkte
	int x2 = (int)ceil(xpos); // Pixelspalte rechts von dem Punkte
	int y1 = (int)floor(ypos); // Pixelreihe �ber dem Punkte
	int y2 = (int)ceil(ypos); // Pixelreihe �ber dem Punkte
	if(x2>=Img->width-1 || y2>=Img->height-1) return CV_RGB(0,0,0);
	double xInfl = (double)xpos - x1; // Einflu� der linken Spalte
	double yInfl = (double)ypos - y1; // Einflu� der oberen Spalte
	
	// abfrage
	CvScalar Val0 = cvGet2D(Img, y1, x1);
	CvScalar Val1 = cvGet2D(Img, y1, x2);
	CvScalar Val2 = cvGet2D(Img, y2, x1);
	CvScalar Val3 = cvGet2D(Img, y2, x2);

	double uRVal0 =  				// Interpolation in der oberen Pixelreihe
		(double)Val0.val[0] * (1-xInfl) +
		(double)Val1.val[0] * (xInfl); 
	
	double uRVal1 =  				// Interpolation in der oberen Pixelreihe
		(double)Val0.val[1] * (1-xInfl) +
		(double)Val1.val[1] * (xInfl); 
	
	double uRVal2 =  				// Interpolation in der oberen Pixelreihe
		(double)Val0.val[2] * (1-xInfl) +
		(double)Val1.val[2] * (xInfl);
	
	double lRVal0 = 				// Interpolation in der unteren Pixelreihe	 
		(double)Val2.val[0] * (1-xInfl) +
		(double)Val3.val[0] * (xInfl);
	
	double lRVal1 = 				// Interpolation in der unteren Pixelreihe	 
		(double)Val2.val[1] * (1-xInfl) +
		(double)Val3.val[1] * (xInfl);

	double lRVal2 = 				// Interpolation in der unteren Pixelreihe	 
		(double)Val2.val[2] * (1-xInfl) +
		(double)Val3.val[2] * (xInfl);
	
	
	CvScalar RetVal;
	RetVal.val[0] = uRVal0 * (1.0-yInfl) + lRVal0 * yInfl;
	RetVal.val[1] = uRVal1 * (1.0-yInfl) + lRVal1 * yInfl;
	RetVal.val[2] = uRVal2 * (1.0-yInfl) + lRVal2 * yInfl;
	RetVal.val[3] = 0;

	return RetVal;
}

/// Funtion to get the gradient of an image point
void GetGradient(IplImage* Image, int x, int y, double& dx, double& dy)
{
	assert(x>0 && x<Image->width-1);
	assert(y>0 && y<Image->height-1);
	
	CvScalar Left = cvGet2D(Image, y, x-1);
	CvScalar Top = cvGet2D(Image, y-1, x);
	CvScalar Base = cvGet2D(Image, y, x);
	CvScalar Right = cvGet2D(Image, y, x+1);
	CvScalar Bottom = cvGet2D(Image, y+1, x);
	dx = (Left.val[2]-Right.val[2])/(Left.val[0]-Right.val[0]);
	dy = (Top.val[2]-Bottom.val[2])/(Top.val[1]-Bottom.val[1]);
}

void GetGradientSobel(IplImage* Image, int x, int y, double& dx, double& dy)
{
	assert(x>0 && x<Image->width-1);
	assert(y>0 && y<Image->height-1);
	
	CvScalar TopLeft = cvGet2D(Image, y-1, x-1);
	CvScalar TopCenter = cvGet2D(Image, y-1, x);
	CvScalar TopRight = cvGet2D(Image, y-1, x+1);
	
	CvScalar Left = cvGet2D(Image, y, x-1);
	CvScalar Right = cvGet2D(Image, y, x+1);

	CvScalar BotLeft = cvGet2D(Image, y+1, x-1);
	CvScalar BotCenter = cvGet2D(Image, y+1, x);
	CvScalar BotRight = cvGet2D(Image, y+1, x+1);
	
	double dx0 = (TopLeft.val[2]-TopRight.val[2])/(TopLeft.val[0]-TopRight.val[0]);
	double dx1 = (Left.val[2]-Right.val[2])/(Left.val[0]-Right.val[0]);
	double dx2 = (BotLeft.val[2]-BotRight.val[2])/(BotLeft.val[0]-BotRight.val[0]);
	
	double dy0 = (BotLeft.val[2]-TopLeft.val[2])/(BotLeft.val[0]-TopLeft.val[0]);
	double dy1 = (BotCenter.val[2]-TopCenter.val[2])/(BotCenter.val[0]-TopCenter.val[0]);
	double dy2 = (BotRight.val[2]-TopRight.val[2])/(BotRight.val[0]-TopRight.val[0]);

	dx = dx0+2.0*dx1+dx2;
	dy = dy0+2.0*dy1+dy2;
}

void GetScaleGradient(IplImage* IntImage, int x, int y, int r, double& Mag, double& Phi)
{
	int le=x-r, ri=x+r+1, to=y-r, bo=y+r+1;
	double A, B, C, D, E, F, G, H, Area = dblsqr(2.0*(double)r+1);
	A = cvGetReal2D(IntImage, to, le);
	B = cvGetReal2D(IntImage, to, x+1);
	C = cvGetReal2D(IntImage, to, ri);
	D = cvGetReal2D(IntImage, y+1, le);
	E = cvGetReal2D(IntImage, y+1, ri);
	F = cvGetReal2D(IntImage, bo, le);
	G = cvGetReal2D(IntImage, bo, x+1);
	H = cvGetReal2D(IntImage, bo, ri);
	double dx = (H-C-2*G+2*B+F-A)/Area;
	double dy = (H-F-2*E+2*D+C-A)/Area;
	Mag = sqrt(dy*dy+dx*dx);
	Phi = atan2(dy, dx); 
}

void GetScaleGradientIter(IplImage* IntImage, int x, int y, int r, double& Mag, double& Phi)
{
	double dxMax=0, dx;
	for(int j=y-r; j<=y+r; j++)
	{
		dx = BoxSum2(IntImage, x-r, x+r, j, j);
		if(fabs(dx)>fabs(dxMax)) dxMax = dx;
	}
	double dyMax=0, dy;
	for(int i=x-r; i<=x+r; i++)
	{
		dy = BoxSum2(IntImage, i, i, y-r, y+1);
		if(fabs(dy)>fabs(dyMax)) dyMax = dy;
	}
	Mag = sqrt(dyMax*dyMax+dxMax*dxMax);
	Phi = atan2(dyMax, dxMax);
}

void GetScaleGradientSobel(IplImage* IntImage, int x, int y, int r, double& Mag, double& Phi)
{
	double s=dblmax(1.0,(double)(2*r+1)/3.0);
	
	double a=cvGetReal2D(IntImage, y-r, x-r);
	double b=cvGetReal2D(IntImage, y-r, cvRound((double)(x-r)+s));
	double c=cvGetReal2D(IntImage, y-r, cvRound((double)(x-r)+2.0*s));
	double d=cvGetReal2D(IntImage, y-r, cvRound((double)(x-r)+3.0*s));
	
	double e=cvGetReal2D(IntImage, cvRound((double)(y-r)+s), x-r);
	double f=cvGetReal2D(IntImage, cvRound((double)(y-r)+s), cvRound((double)(x-r)+s));
	double g=cvGetReal2D(IntImage, cvRound((double)(y-r)+s), cvRound((double)(x-r)+2.0*s));
	double h=cvGetReal2D(IntImage, cvRound((double)(y-r)+s), cvRound((double)(x-r)+3.0*s));
	
	double i=cvGetReal2D(IntImage, cvRound((double)(y-r)+2.0*s), x-r);
	double j=cvGetReal2D(IntImage, cvRound((double)(y-r)+2.0*s), cvRound((double)(x-r)+s));
	double k=cvGetReal2D(IntImage, cvRound((double)(y-r)+2.0*s), cvRound((double)(x-r)+2.0*s));
	double l=cvGetReal2D(IntImage, cvRound((double)(y-r)+2.0*s), cvRound((double)(x-r)+3.0*s));

	double m=cvGetReal2D(IntImage, cvRound((double)(y-r)+3.0*s), x-r);
	double n=cvGetReal2D(IntImage, cvRound((double)(y-r)+3.0*s), cvRound((double)(x-r)+s));
	double o=cvGetReal2D(IntImage, cvRound((double)(y-r)+3.0*s), cvRound((double)(x-r)+2.0*s));
	double p=cvGetReal2D(IntImage, cvRound((double)(y-r)+3.0*s), cvRound((double)(x-r)+3.0*s));
	
	double a00 = f-b-e+a;
	double a01 = g-c-f+b;
	double a02 = h-d-g+c;
	
	double a10 = j-f-i+e;
	double a12 = l-h-k+g;
	
	double a20 = n-j-m+i;
	double a21 = o-k-n+j;
	double a22 = p-l-o+k;

	double dx = -a00-2*a10-a20+a02+2*a12+a22;
	double dy = -a00-2*a01-a02+a02+2*a21+a22;

	Mag = sqrt(dy*dy+dx*dx);
	Phi = atan2(dy, dx); 
}

/*
void GetScaleGradientInterpolated(IplImage* IntImage, int x, int y, int r, double& Mag, double& Phi)
{
	double lm, rm, tm, bm;
	double la, ra, ta, ba;
	GetScaleGradient(IntImage, x, y, r+1, lm, la);
	GetScaleGradient(IntImage, x, y, r+2, rm, ra);
	GetScaleGradient(IntImage, x, y, r+3, tm, ta);
	GetScaleGradient(IntImage, x, y, r+4, bm, ba);
	//double Sum = lm+rm+tm+bm;
	//double dx = (sin(la)*lm+sin(ra)*rm+sin(ta)*tm+sin(ba)*bm)/Sum;
	//double dy = (cos(la)*lm+cos(ra)*rm+cos(ta)*tm+cos(ba)*bm)/Sum;
	double dx = sin(la);//+sin(ra)+sin(ta)+sin(ba);
	double dy = cos(la);//+cos(ra)+cos(ta)+cos(ba);
	
	Mag = sqrt(dy*dy+dx*dx);
	Phi = atan2(dy, dx);
}
*/

int GetNormal(IplImage* Image, int x, int y, Point3Dbl& Res, int k)
{
	/// Assert, that we are not exceeding any boundaries.
	assert(x-k>=0 && x+k<Image->width);
	assert(y-k>=0 && y+k<Image->height);
	
	/// Read out the neighbors
	CvScalar Left = cvGet2D(Image, y, x-k);
	if(Left.val[0]==0.0 && Left.val[1]==0.0 && Left.val[2]==0.0) return RET_FAILED; 
	CvScalar Top = cvGet2D(Image, y-k, x);
	if(Top.val[0]==0.0 && Top.val[1]==0.0 && Top.val[2]==0.0) return RET_FAILED;
	CvScalar Right = cvGet2D(Image, y, x+k);
	if(Right.val[0]==0.0 && Right.val[1]==0.0 && Right.val[2]==0.0) return RET_FAILED;
	CvScalar Bottom = cvGet2D(Image, y+k, x);
	if(Bottom.val[0]==0.0 && Bottom.val[1]==0.0 && Bottom.val[2]==0.0) return RET_FAILED;

	/// Get the gradient components
	Point3Dbl vx, vy, pr, pl, pt, pb;
	pr.m_x = Right.val[0]; pr.m_y = Right.val[1]; pr.m_z = Right.val[2];
	pl.m_x  = Left.val[0]; pl.m_y = Left.val[1]; pl.m_z = Left.val[2];
	vx.SubVec(pr, pl);
	pt.m_x = Top.val[0]; pt.m_y = Top.val[1]; pt.m_z = Top.val[2];
	pb.m_x = Bottom.val[0]; pb.m_y = Bottom.val[1]; pb.m_z = Bottom.val[2];
	vy.SubVec(pb, pt);
	
	if(vx.AbsVal()==0 || vy.AbsVal()==0) return RET_FAILED;

	Res.VectorProd(vy, vx);
	Res.Normalize();

	return RET_OK;
}

//void GetNormal2(IplImage* Image, int x, int y, Point3Dbl& Res, int k)
//{
//	/// Assert, that we are not exceeding any boundaries.
//	assert(x-k>=0 && x+k<Image->width);
//	assert(y-k>=0 && y+k<Image->height);
//	
//	/// Read out the neighbors
//	CvScalar TopLeft = cvGet2D(Image, y-k, x-k);
//	CvScalar TopCenter = cvGet2D(Image, y-k, x);
//	CvScalar TopRight = cvGet2D(Image, y-k, x+k);
//
//	CvScalar Left = cvGet2D(Image, y, x-k);
//	CvScalar Right = cvGet2D(Image, y, x+k);
//
//	CvScalar BotLeft = cvGet2D(Image, y+k, x-k);
//	CvScalar BotCenter = cvGet2D(Image, y+k, x);
//	CvScalar BotRight = cvGet2D(Image, y+k, x+k);
//
//	/// Get the gradient components
//	double dx0x = TopRight.val[0]-TopLeft.val[0];
//	double dx0y = TopRight.val[1]-TopLeft.val[1];
//	double dx0z = TopRight.val[2]-TopLeft.val[2];
//	double dx1x = Right.val[0]-Left.val[0];
//	double dx1y = Right.val[1]-Left.val[1];
//	double dx1z = Right.val[2]-Left.val[2];
//	double dx2x = BotRight.val[0]-BotLeft.val[0];
//	double dx2y = BotRight.val[1]-BotLeft.val[1];
//	double dx2z = BotRight.val[2]-BotLeft.val[2];
//	double dy0x = BotLeft.val[0]-TopLeft.val[0];
//	double dy0y = BotLeft.val[1]-TopLeft.val[1];
//	double dy0z = BotLeft.val[2]-TopLeft.val[2];
//	double dy1x = BotCenter.val[0]-TopCenter.val[0];
//	double dy1y = BotCenter.val[1]-TopCenter.val[1];
//	double dy1z = BotCenter.val[2]-TopCenter.val[2];
//	double dy2x = BotRight.val[0]-TopRight.val[0];
//	double dy2y = BotRight.val[1]-TopRight.val[1];
//	double dy2z = BotRight.val[2]-TopRight.val[2];
//	
//	Point3Dbl DX(0.25*dx0x+0.5*dx1x+0.25*dx2x, 0.25*dx0y+0.5*dx1y+0.25*dx2y, 0.25*dx0z+0.5*dx1z+0.25*dx2z);
//	Point3Dbl DY(0.25*dy0x+0.5*dy1x+0.25*dy2x, 0.25*dy0y+0.5*dy1y+0.25*dy2y, 0.25*dy0z+0.5*dy1z+0.25*dy2z);
//
//	//Res.VectorProd(DX, DY);
//	Res.VectorProd(DY, DX);
//	Res.Normalize();
//}

void GetNormalSobel(IplImage* Image, int x, int y, Point3Dbl& Res)
{	
	double dx, dy;
	GetGradientSobel(Image, x, y, dx, dy);	
	Res.VectorProd(dx, dy);
	Res.Normalize();
}

void SetBackgroundNeutral(IplImage* Image, CvScalar MaskVal)
{
	/// Get mean
	CvScalar Mean=CV_RGB(0,0,0);
	double Sum=0.0;
	int i,j;
	for(j=0; j<Image->height; j++)
		for(i=0; i<Image->width; i++)
		{
			CvScalar Val=cvGet2D(Image, j, i);
			if(Val.val[0]==MaskVal.val[0] && Val.val[1]==MaskVal.val[1] && Val.val[2]==MaskVal.val[2]) continue;
			else
			{
				Mean.val[0]+=Val.val[0];
				Mean.val[1]+=Val.val[1];
				Mean.val[2]+=Val.val[2];
				Sum+=1.0;
			}
		}
	Mean.val[0]/=Sum;
	Mean.val[1]/=Sum;
	Mean.val[2]/=Sum;

	/// Change pixels
	for(j=0; j<Image->height; j++)
		for(i=0; i<Image->width; i++)
		{
			CvScalar Val=cvGet2D(Image, j, i);
			if(Val.val[0]==MaskVal.val[0] && Val.val[1]==MaskVal.val[1] && Val.val[2]==MaskVal.val[2]) cvSet2D(Image, j, i, Mean);
		}
}

void TransformCoordinateImage(IplImage* CoordImage, Frame& F, bool Invert)
{
	/// Change pixels
	for(int j=0; j<(int)CoordImage->height; j++)
	{
		for(int i=0; i<(int)CoordImage->width; i++)
		{
			CvScalar Val=cvGet2D(CoordImage, j, i);
			if(Val.val[0]==0 && Val.val[1]==0 && Val.val[2]==0) continue;
			Point3Dbl v(Val.val[0], Val.val[1], Val.val[2]);
			if(!Invert) F.ToWorld(v);
			else F.ToFrame(v);
			cvSet2D(CoordImage, j, i, cvScalar(v.m_x, v.m_y, v.m_z));
		}
	}
}

/// 

/*
// Grauwert eines nicht gerasterten Pixels (linear interpoliert)
// xsize : Breite des gesamten Bildes
// xpos, ypos : Position zu berechnenden Farbwertes

double InterpolRealValue(IplImage* Img, double xpos, double ypos){

	int x1 = (int)floor(xpos); // Pixelspalte links von dem Punkte
	int x2 = (int)ceil(xpos); // Pixelspalte rechts von dem Punkte
	int y1 = (int)floor(ypos); // Pixelreihe �ber dem Punkte
	int y2 = (int)ceil(ypos); // Pixelreihe �ber dem Punkte
	double xInfl = (double)xpos - x1; // Einflu� der linken Spalte
	double yInfl = (double)ypos - y1; // Einflu� der oberen Spalte
	// abfrage 
	double uRVal =  				// Interpolation in der oberen Pixelreihe
		(double)cvGetReal2D(Img, y1, x1) * (1-xInfl) +
		(double)cvGetReal2D(Img, y1, x2) * (xInfl); 
				
	double lRVal = 				// Interpolation in der unteren Pixelreihe	 
		(double)cvGetReal2D(Img, y2, x1) * (1-xInfl) +
		(double)cvGetReal2D(Img, y2, x2) * (xInfl); 
		
	double retVal = uRVal * (1.0-yInfl) + lRVal * yInfl; 
								// zwischen den Reihen interpolieren
	return retVal;
}

double InterpolChannelValue(IplImage* Img, double xpos, double ypos, int c){

	int x1 = (int)floor(xpos); // Pixelspalte links von dem Punkte
	int x2 = (int)ceil(xpos); // Pixelspalte rechts von dem Punkte
	int y1 = (int)floor(ypos); // Pixelreihe �ber dem Punkte
	int y2 = (int)ceil(ypos); // Pixelreihe �ber dem Punkte
	double xInfl = (double)xpos - x1; // Einflu� der linken Spalte
	double yInfl = (double)ypos - y1; // Einflu� der oberen Spalte
	// abfrage 
	double uRVal =  				// Interpolation in der oberen Pixelreihe
		(double)GetChannelVal(Img, y1, x1, c) * (1-xInfl) +
		(double)GetChannelVal(Img, y1, x2, c) * (xInfl); 
				
	double lRVal = 				// Interpolation in der unteren Pixelreihe	 
		(double)GetChannelVal(Img, y2, x1, c) * (1-xInfl) +
		(double)GetChannelVal(Img, y2, x2, c) * (xInfl); 
		
	double retVal = uRVal * (1.0-yInfl) + lRVal * yInfl; 
								// zwischen den Reihen interpolieren
	return retVal;
}	

CvScalar InterpolValue(IplImage* Img, double xpos, double ypos)
{

	int x1 = (int)floor(xpos); // Pixelspalte links von dem Punkte
	int x2 = (int)ceil(xpos); // Pixelspalte rechts von dem Punkte
	int y1 = (int)floor(ypos); // Pixelreihe �ber dem Punkte
	int y2 = (int)ceil(ypos); // Pixelreihe �ber dem Punkte
	double xInfl = (double)xpos - x1; // Einflu� der linken Spalte
	double yInfl = (double)ypos - y1; // Einflu� der oberen Spalte
	// abfrage
	CvScalar Val0 = cvGet2D(Img, y1, x1);
	CvScalar Val1 = cvGet2D(Img, y1, x2);
	CvScalar Val2 = cvGet2D(Img, y2, x1);
	CvScalar Val3 = cvGet2D(Img, y2, x2);

	double uRVal0 =  				// Interpolation in der oberen Pixelreihe
		(double)Val0.val[0] * (1-xInfl) +
		(double)Val1.val[0] * (xInfl); 
	
	double uRVal1 =  				// Interpolation in der oberen Pixelreihe
		(double)Val0.val[1] * (1-xInfl) +
		(double)Val1.val[1] * (xInfl); 
	
	double uRVal2 =  				// Interpolation in der oberen Pixelreihe
		(double)Val0.val[2] * (1-xInfl) +
		(double)Val1.val[2] * (xInfl);
	
	double lRVal0 = 				// Interpolation in der unteren Pixelreihe	 
		(double)Val2.val[0] * (1-xInfl) +
		(double)Val3.val[0] * (xInfl);
	
	double lRVal1 = 				// Interpolation in der unteren Pixelreihe	 
		(double)Val2.val[1] * (1-xInfl) +
		(double)Val3.val[1] * (xInfl);

	double lRVal2 = 				// Interpolation in der unteren Pixelreihe	 
		(double)Val2.val[2] * (1-xInfl) +
		(double)Val3.val[2] * (xInfl);
	
	
	CvScalar RetVal;
	RetVal.val[0] = uRVal0 * (1.0-yInfl) + lRVal0 * yInfl;
	RetVal.val[1] = uRVal1 * (1.0-yInfl) + lRVal1 * yInfl;
	RetVal.val[2] = uRVal2 * (1.0-yInfl) + lRVal2 * yInfl;

	return RetVal;
}	


// Schneidet ein gedrehtes quadratisches Frame aus einem Bild.
// inputImage : das Ausgangsbild
// returnImage : das resultierende Frame
// imageXSize, imageYSize : Gr��e des Ausgangsbildes
// frameSize : Kantenl�nge des auszuschneidenden Quadrats
// frameRot : Drehung des Frames
// frameXPos, frameYPos : linke obere Ecke des Frames

void TruncRotFrame(IplImage* Img, IplImage*& RetImg, int frameSize, double frameRot,
				double frameXPos, double frameYPos)
{

	
	RetImg = cvCreateImage(cvSize(frameSize, frameSize), IPL_DEPTH_8U, 1);
	double sinRot = sinf(frameRot);
	double cosRot = cosf(frameRot);

	for(int xi = -frameSize/2; xi < frameSize/2; xi++)
	{
		for(int yi = -frameSize/2; yi < frameSize/2; yi++)
		{  // Alle Pixel im Frame durchgehen
			double xpos = frameXPos + cosRot*xi - sinRot*yi; // X-Position auf dem Ausgangsbild
			double ypos = frameYPos + cosRot*yi + sinRot*xi; // Y-Position auf dem Ausgangsbild
			cvSetReal2D(RetImg, yi+frameSize/2, xi+frameSize/2, InterpolRealValue(Img, xpos, ypos));
				// den Interpolierten Wert in das neue Bild schreiben
		}
	}

}

// copies a rotated image into another image
int CopyImageFrame(IplImage* Input, IplImage* Output, CvPoint Center, double Angle, int Fill)
{
	double SinRot = sinf(Angle);
	double CosRot = cosf(Angle);
	
	for(int xi = -Output->width/2; xi < Output->width/2; xi++)
	{
		for(int yi = -Output->height/2; yi < Output->height/2; yi++)
		{  // Alle Pixel im Frame durchgehen
			
			double xpos = Center.x + CosRot*xi - SinRot*yi; // X-Position auf dem Ausgangsbild
			double ypos = Center.y + CosRot*yi + SinRot*xi; // Y-Position auf dem Ausgangsbild
			
			if(Input->nChannels==1)
			{
				if(xpos<0 || xpos>=Input->width || ypos<0 || ypos>=Input->height)
					cvSetReal2D(Output, yi+Input->height/2, xi+Input->width/2, Fill);
				else
					cvSetReal2D(Output, yi+Input->height/2, xi+Input->width/2, InterpolRealValue(Input, xpos, ypos));
			}
			else if(Input->nChannels==3)
			{
				if(xpos<0 || xpos>=Input->width || ypos<0 || ypos>=Input->height)
					cvSet2D(Output, yi+Output->height/2, xi+Output->width/2, CV_RGB(Fill, Fill, Fill));
				else
					cvSet2D(Output, yi+Output->height/2, xi+Output->width/2, InterpolValue(Input, xpos, ypos));
			}
			else return -1;
		}
	}
	return 0;
}
*/
/////////////////////////
/*
void GetGlobalCenter(IplImage* Image, int& IMin, int& JMin, double Cut)
{
	assert(Image->nChannels == 1);
	int width = Image->width;
	int height = Image->height;
	
	int j0 = (int)floor((double)height*Cut);
	int j1 = (int)floor((double)height*(1.0-Cut));
	int i0 = (int)floor((double)width*Cut);
	int i1 = (int)floor((double)width*(1.0-Cut));
	
	double ISum, JSum;
	double JSumMin = DBL_MAX;
	double ISumMin = DBL_MAX;
	int i=-1, j=-1;
	
	for(j=j0; j<j1; j++)
	{
		ISum=0.0;
		for(i=i0; i<i1; i++)
		{
			ISum+=cvGetReal2D(Image, j, i);
		}
		if(ISum < ISumMin)
		{
			ISumMin = ISum;
			JMin = j;
			//std::cout << "Sum min: " << ISumMin << std::endl;
			//getchar();
		}
	}
	
	for(i=i0; i<i1; i++)
	{
		JSum=0.0;
		for(j=j0; j<j1; j++)
		{
			JSum+=cvGetReal2D(Image, j, i);
		}
		if(JSum < JSumMin)
		{
			JSumMin = JSum;
			IMin = i;
		}
	}
	//IMin = i0;
	//JMin = j0;
	
}
*/

/*
// funtion to get the gradient of an image point
void GetGradient(IplImage* Image, int x, int y, double& dx, double& dy)
{
	assert(x>0 && x<Image->width-1);
	assert(y>0 && y<Image->height-1);
	
	CvScalar Left = cvGet2D(Image, y, x-1);
	CvScalar Top = cvGet2D(Image, y-1, x);
	CvScalar Base = cvGet2D(Image, y, x);
	CvScalar Right = cvGet2D(Image, y, x+1);
	CvScalar Bottom = cvGet2D(Image, y+1, x);
	dx = (Left.val[2]-Right.val[2])/(Left.val[0]-Right.val[0]);
	dy = (Top.val[2]-Bottom.val[2])/(Top.val[1]-Bottom.val[1]);
}
*/

/*
// funtion to get the surface normal of an image point
void GetNormal(IplImage* Image, int x, int y, int k, Point3& Res)
{
	assert(x-k>=0 && x+k<Image->width);
	assert(y-k>=0 && y+k<Image->height);
	
	CvScalar Left = cvGet2D(Image, y, x-k);
	CvScalar Top = cvGet2D(Image, y-k, x);
	CvScalar Base = cvGet2D(Image, y, x);
	CvScalar Right = cvGet2D(Image, y, x+k);
	CvScalar Bottom = cvGet2D(Image, y+k, x);

	Point3 vx;
	Point3 pr = Set(Right.val[0], Right.val[1], Right.val[2]);
	Point3 pl = Set(Left.val[0], Left.val[1], Left.val[2]);
	SubVec(pr, pl, vx); 
	Point3 vy;
	Point3 pt = Set(Top.val[0], Top.val[1], Top.val[2]);
	Point3 pb = Set(Bottom.val[0], Bottom.val[1], Bottom.val[2]);
	SubVec(pt, pb, vy); 
	Point3 CP;
	VectorProd(vx, vy, CP);
	Normalize(CP);
	CopyPt(CP, Res);
}
*/

/*
}void SaveFloatImage(char* Name, IplImage* Img)
{
        IplImage* SImg = cvCreateImage(cvGetSize(Img), IPL_DEPTH_8U, 1);
        double Min, Max;
        cvMinMaxLoc(Img, &Min, &Max);
        cvConvertScale(Img, SImg, 255.0/(Max-Min), -Min*255.0/(Max-Min));
        cvSaveImage(Name, SImg);
        cvReleaseImage(&SImg);
}har* Name, IplImage* Img)
{
        IplImage* SImg = cvCreateImage(cvGetSize(Img), IPL_DEPTH_8U, 1);
        double Min, Max;
        cvMinMaxLoc(Img, &Min, &Max);
        cvConvertScale(Img, SImg, 255.0/(Max-Min), -Min*255.0/(Max-Min));
        cvSaveImage(Name, SImg);
        cvReleaseImage(&SImg);
}
	cvMinMaxLoc(Source, &Min, &Max);
	cvConvertScaleAbs(Source, Dest, 255.0/(Max-Min), -(Min*255.0/(Max-Min))*Fac);
}

void SaveFloatImage(char* Name, IplImage* Img)
{
        IplImage* SImg = cvCreateImage(cvGetSize(Img), IPL_DEPTH_8U, 1);
        double Min, Max;
        cvMinMaxLoc(Img, &Min, &Max);
        cvConvertScale(Img, SImg, 255.0/(Max-Min), -Min*255.0/(Max-Min));
        cvSaveImage(Name, SImg);
        cvReleaseImage(&SImg);
}
*/
/*
int LoadImages(std::string Name, std::string Path,
			   IplImage** Range, IplImage** Intensity,
			   IplImage** Coords, IplImage** Color)
{
	//// all should be non-created and set to NULL
	assert(Range==NULL && Intensity==NULL && Coords==NULL && Color==NULL);
	setlocale(LC_NUMERIC, "C"); // ensure to use points - not commas
	
	int Ret=0;
	char FullName[1024];
	
	sprintf(FullName, "%s%sRange.xml", Path.c_str(), Name.c_str());
	//if(Range!=NULL) {cvReleaseImage(&Range); Range=NULL;}
	*Range = (IplImage*)cvLoad(FullName);
	if(*Range==NULL) Ret-=1;

	sprintf(FullName, "%s%sIntensity.xml", Path.c_str(), Name.c_str());
	//if(Intensity!=NULL) {cvReleaseImage(&Intensity); Intensity=NULL;}
	*Intensity = (IplImage*)cvLoad(FullName);
	if(*Intensity==NULL) Ret-=1;

	sprintf(FullName, "%s%sCoords.xml", Path.c_str(), Name.c_str());
	//if(Coords!=NULL) {cvReleaseImage(&Coords); Coords=NULL;}
	*Coords = (IplImage*)cvLoad(FullName);
	if(*Coords==NULL) Ret-=1;

	sprintf(FullName, "%s%sColor.bmp", Path.c_str(), Name.c_str());
	//if(Color!=NULL) {cvReleaseImage(&Color); Color=NULL;}
	*Color = cvLoadImage(FullName);
	if(*Color==NULL) Ret-=1;

	return Ret;
}

int SaveImages(std::string Name, std::string Path,
			   IplImage* Range, IplImage* Intensity,
			   IplImage* Coords, IplImage* Color)
{
	setlocale(LC_NUMERIC, "C"); // ensure to use points - not commas

	char FullName[1024];
	
	if(Range!=NULL)
	{
		sprintf(FullName, "%s%sRange.xml", Path.c_str(), Name.c_str());
		cvSave(FullName, Range);
	}
	
	if(Intensity!=NULL)
	{
		sprintf(FullName, "%s%sIntensity.xml", Path.c_str(), Name.c_str());
		cvSave(FullName, Intensity);
	}

	if(Coords!=NULL)
	{
		sprintf(FullName, "%s%sCoords.xml", Path.c_str(), Name.c_str());
		cvSave(FullName, Coords);
	}
	
	if(Color!=NULL)
	{
		sprintf(FullName, "%s%sColor.bmp", Path.c_str(), Name.c_str());
		cvSaveImage(FullName, Color);
	}
	
	return 0;
}
*/

/*
double FastFeatExtract::InterpolateExtr(double x0, double y0, double x1, double y1, double x2, double y2)
{
	double a1 = (y0-y1)/(x0-x1);
	double a2 = (a1-(y1-y2)/(x1-x2))/(x0-x2);
	return 0.5*(x0+x1-(a1/a2));
}
*/


ImageTriple::ImageTriple(IplImage* SRXYZImg, IplImage* SRIntenImg, IplImage* ColorImg)
{
	/// Release old images
	Release();

	/// Make clones
	m_SRXYZImg=cvCloneImage(SRXYZImg);
	m_SRIntenImg=cvCloneImage(SRIntenImg);
	m_ColorImg=cvCloneImage(ColorImg);	
}

ImageTriple::~ImageTriple()
{
	Release();
}

void ImageTriple::Release()
{
	if(m_SRXYZImg!=NULL) cvReleaseImage(&m_SRXYZImg);
	if(m_SRIntenImg!=NULL) cvReleaseImage(&m_SRIntenImg);
	if(m_ColorImg!=NULL) cvReleaseImage(&m_ColorImg);
}

unsigned long ImageTriple::Load(std::string Name)
{
	Release();
	setlocale(LC_NUMERIC, "C"); 
	std::stringstream n0, n1, n2;
	n0 << Name << "_Coord.xml";
	std::cout << n0.str();
	m_SRXYZImg = (IplImage*)cvLoad(n0.str().c_str());
	n1 << Name << "_Inten.bmp";
	m_SRIntenImg = cvLoadImage(n1.str().c_str());
	n2 << Name << "_Color.bmp";
	m_ColorImg = cvLoadImage(n2.str().c_str());
	if(m_SRXYZImg==NULL || m_SRIntenImg==NULL || m_ColorImg==NULL) return RET_FAILED;
	else return RET_OK;
}

unsigned long ImageTriple::Save(std::string Name)
{
	setlocale(LC_NUMERIC, "C"); 
	std::stringstream n0, n1, n2;
	n0 << Name << "_Coord.xml";
	cvSave(n0.str().c_str(), m_SRXYZImg);
	n1 << Name << "_Inten.bmp";
	cvSaveImage(n1.str().c_str(), m_SRIntenImg);
	n2 << Name << "_Color.bmp";
	cvSaveImage(n2.str().c_str(), m_ColorImg);
	return RET_OK;
}

void ImageTriple::Delete(std::string Name)
{
	setlocale(LC_NUMERIC, "C");
	std::stringstream n0, n1, n2;
	n0 << Name << "_Coord.xml";
	n1 << Name << "_Inten.bmp";
	n2 << Name << "_Color.bmp";
	remove(n0.str().c_str());
	remove(n1.str().c_str());
	remove(n2.str().c_str());
}

void ImageTripleList::PushBack(IplImage* SRXYZImg, IplImage* SRIntenImg, IplImage* ColorImg)
{
	push_back(ImageTriple(SRXYZImg, SRIntenImg, ColorImg));
}

unsigned long ImageTripleList::Load(std::string FileName)
{
	return RET_OK;
}

unsigned long ImageTripleList::Save(std::string FileName)
{
	return RET_OK;
}

void ImageTripleList::Clear()
{
	
}

void TransformPoint(double x_in, double y_in, double z_in, double& x_out, double& y_out, double& z_out, const CvMat* R, const CvMat* t)
{
	double r00 = cvGetReal1D(R, 0), r01 = cvGetReal1D(R, 1), r02 = cvGetReal1D(R, 2);
	double r10 = cvGetReal1D(R, 3), r11 = cvGetReal1D(R, 4), r12 = cvGetReal1D(R, 5);
	double r20 = cvGetReal1D(R, 6), r21 = cvGetReal1D(R, 7), r22 = cvGetReal1D(R, 8);
	double t0 = cvGetReal1D(t, 0), t1 = cvGetReal1D(t, 1), t2 = cvGetReal1D(t, 2);

	x_out = r00 * x_in + r01 * y_in + r02 * z_in + t0;
	y_out = r10 * x_in + r11 * y_in + r12 * z_in + t1;
	z_out = r20 * x_in + r21 * y_in + r22 * z_in + t2;
} 

void TransformPointInverse(double x_in, double y_in, double z_in, double& x_out, double& y_out, double& z_out, const CvMat* R, const CvMat* t)
{
	double r00 = cvGetReal1D(R, 0), r01 = cvGetReal1D(R, 1), r02 = cvGetReal1D(R, 2);
	double r10 = cvGetReal1D(R, 3), r11 = cvGetReal1D(R, 4), r12 = cvGetReal1D(R, 5);
	double r20 = cvGetReal1D(R, 6), r21 = cvGetReal1D(R, 7), r22 = cvGetReal1D(R, 8);
	double t0 = cvGetReal1D(t, 0), t1 = cvGetReal1D(t, 1), t2 = cvGetReal1D(t, 2);

	double div = r00*(r12*r21-r11*r22)+r01*(r10*r22-r12*r20)+r02*(r11*r20-r10*r21);
	
	x_out = (r02*(r11*(z_in-t2)-r21*y_in+r21*t1)+r01*(r12*(t2-z_in)+r22*y_in-r22*t1)+(r12*r21-r11*r22)*x_in+(r11*r22-r12*r21)*t0)/div;
	y_out = (r02*(r10*(z_in-t2)-r20*y_in+r20*t1)+r00*(r12*(t2-z_in)+r22*y_in-r22*t1)+(r12*r20-r10*r22)*x_in+(r10*r22-r12*r20)*t0)/div;
	z_out = (r01*(r10*(z_in-t2)-r20*y_in+r20*t1)+r00*(r11*(t2-z_in)+r21*y_in-r21*t1)+(r11*r20-r10*r21)*x_in+(r10*r21-r11*r20)*t0)/div;
} 

void TransformPointRodrigues(double x_in, double y_in, double z_in, double& x_out, double& y_out, double& z_out, const CvMat* r_vec, const CvMat* t)
{
	CvMat* R = cvCreateMat(3, 3, CV_32F);
	cvRodrigues2(r_vec, R);
	TransformPoint(x_in, y_in, z_in,  x_out, y_out, z_out, R, t);
	cvReleaseMat(&R);
}

void TransformPointRodriguesInverse(double x_in, double y_in, double z_in, double& x_out, double& y_out, double& z_out, const CvMat* r_vec, const CvMat* t)
{
	CvMat R = cvMat(3, 3, CV_64F);
	cvRodrigues2(r_vec, &R);
	TransformPointInverse(x_in, y_in, z_in,  x_out, y_out, z_out, &R, t);
} 

void GetCorrectedUV(double u, double v, double fx, double cx, double fy, double cy, double w, double h,
					double k0, double k1, double k2, double k3, double& u_dot, double& v_dot)
{

	double x = (u-cx)/fx;
	double y = (v-cy)/fy;

	double r2, r4, r6, a1, a2, a3, cdist;
	r2 = x*x + y*y;
	r4 = r2*r2;
	r6 = r4*r2;
	a1 = 2*x*y;
	a2 = r2 + 2*x*x;
	a3 = r2 + 2*y*y;
	
	cdist = 1 + k0*r2 + k1*r4;
	double x_dot = x*cdist + k2*a1 + k3*a2;
	double y_dot = y*cdist + k2*a3 + k3*a1;
	
	/// Get the projection sizes x_dot and y_dot
	u_dot = fx*x_dot + w/2.0;
	v_dot = fy*y_dot + h/2.0;
	if(u_dot<0) u_dot=0;
	else if(u_dot>=w) u_dot = w-1;
	if(v_dot<0) v_dot=0;
	else if(v_dot>=h) v_dot = h-1;
}

void PerspectiveProjection(double x_in, double y_in, double z_in, double& u_out, double& v_out, const CvMat* A, const CvMat* distCoeffs)
{
	double fx = cvGetReal1D(A, 0);
	double fy = cvGetReal1D(A, 4);
	double cx = cvGetReal1D(A, 2);
	double cy = cvGetReal1D(A, 5);
	
	double x,y,z;
	z = z_in ? 1.0/z_in : 1;
	x = x_in * z;
	y = y_in * z;
        	
	double xd=x, yd=y;
	
	if(distCoeffs!=0)
	{
		double r2, r4, r6, a1, a2, a3, cdist;

		r2 = x*x + y*y;
		r4 = r2*r2;
		r6 = r4*r2;
		a1 = 2*x*y;
		a2 = r2 + 2*x*x;
		a3 = r2 + 2*y*y;
	
		double k0 = cvGetReal1D(distCoeffs, 0);
		double k1 = cvGetReal1D(distCoeffs, 1);
		double k2 = cvGetReal1D(distCoeffs, 2);
		double k3 = cvGetReal1D(distCoeffs, 3);
		
		cdist = 1 + k0*r2 + k1*r4;
        xd = x*cdist + k2*a1 + k3*a2;
        yd = y*cdist + k2*a3 + k3*a1;
	}

	u_out = xd*fx + cx;
	v_out = yd*fy + cy;
}
/*
void PerspectiveProjectionInverseUndistorted(int u_in, int v_in, double z_in, double& x_out, double& y_out, const CvMat* A)
{
	double a[9], fx, fy, cx, cy;
	CvMat _a = cvMat(3, 3, CV_64F, a);
	cvConvert(A, &_a );

	fx = a[0]; fy = a[4];
	cx = a[2]; cy = a[5];
	
	x_out = (u_in - cx) * z_in/fx;
	y_out = (u_in - cx) * z_in/fy;
}
*/
//void GetXYZImageFromRangeImage(IplImage* RangeImage,
//							   IplImage* AZ, IplImage* BZ, IplImage* CZ,
//							   IplImage* X, IplImage* Y, IplImage* xyzImage)
//{
//	/// Get the z-image
//	IplImage* xyzImgTmp = cvCreateImage(cvGetSize(RangeImage), IPL_DEPTH_32F, 3);
//	int h = RangeImage->height, w = RangeImage->width;
//	for(int j=0; j<h; j++)
//	{
//		for(int i=0; i<w; i++)
//		{
//			/// Get the z value
//			double az = cvGetReal2D(AZ, j, i);
//			double bz = cvGetReal2D(BZ, j, i);
//			double cz = cvGetReal2D(CZ, j, i);
//			double R = cvGetReal2D(RangeImage, j, i);
//			double z = az * R * R + bz * R + cz ;
//
//			/// Get the x and y values
//			double x = cvGetReal2D(X, j, i) * z;
//			double y = cvGetReal2D(Y, j, i) * z;
//
//			/// Set the value
//			cvSet2D(xyzImgTmp, j, i, cvScalar(x, y, z));
//
//			//std::cout << "xyz " << x << " " << y << " " << z << "\n"; 
//		}
//	}
//	
//	/// Copy and resize
//	cvResize(xyzImgTmp, xyzImage);
//	cvReleaseImage(&xyzImgTmp);
//}

void UndistortRangerImages(IplImage* rangeImage, IplImage* intenImage, IplImage* mapX, IplImage* mapY)
{

	IplImage* tmpRange = cvCreateImage(cvGetSize(rangeImage), rangeImage->depth, rangeImage->nChannels);
	IplImage* tmpInten = cvCreateImage(cvGetSize(intenImage), intenImage->depth, intenImage->nChannels);

	int i, j, I, J;//, K;
	for(j=0; j<rangeImage->height; j++)
	{
		for(i=0; i<rangeImage->width; i++)
		{
			I = (int)((float*)(mapX->imageData + j*mapX->widthStep))[i];
			J = (int)((float*)(mapY->imageData + j*mapY->widthStep))[i];

			//(tmpRange->imageData + K)[I*tmpRange->nChannels] = (rangeImage->imageData + K)[I*rangeImage->nChannels];
			//(tmpInten->imageData + K)[I*tmpInten->nChannels] = (intenImage->imageData + K)[I*intenImage->nChannels];
			
			((float*)(tmpRange->imageData + j*tmpRange->widthStep))[i] = ((float*)(rangeImage->imageData + J*rangeImage->widthStep))[I];
			((float*)(tmpInten->imageData + j*tmpInten->widthStep))[i] = ((float*)(intenImage->imageData + J*intenImage->widthStep))[I];
			
			//double rVal = cvGetReal2D(rangeImage, J, I);//(double)(rangeImage->imageData + J*rangeImage->widthStep)[I*rangeImage->nChannels];//cvGetReal2D(rangeImage, J, I);
			//double iVal = cvGetReal2D(intenImage, J, I);//(double)(intenImage->imageData + J*intenImage->widthStep)[I*intenImage->nChannels];//cvGetReal2D(intenImage, J, I);
			//cvSetReal2D(tmpRange, j, i, rVal);
			//cvSetReal2D(tmpInten, j, i, iVal);
		}
	}
	
	cvCopy(tmpRange, rangeImage);
	cvCopy(tmpInten, intenImage);

	cvReleaseImage(&tmpRange);
	cvReleaseImage(&tmpRange);
}

void ForwardTrafoSR(int u, int v, int t, double& x, double& y, double& z, double f, double cx, double cy, double k, double l, double m)
{
	/// Get the projection sizes
	double X = ((double)u - cx) / f;
	double Y = ((double)v - cy) / f;

	/// Get the measured range value and the estimated range value
	double estRange = k * t * t + l * t + m ;
	double rConst = sqrt( 1 + X * X + Y * Y );

	/// Get the z-value by dividing thorugh the square root term
	z = estRange/rConst;

	/// Get x and y
	x = X * z;
	y = Y * z;
}

void GetXYZImageFromUndistortedRangeImage(IplImage* rangeImage, double f, double k, double l, double m, IplImage* xyzImage)
{
	double x, y, z;
	int J, I;
	for(int j=0; j<rangeImage->height; j++)
	{
		J = j*xyzImage->widthStep;
		for(int i=0; i<rangeImage->width; i++)
		{
			/// Get the projection sizes
			//double X = (i - (double)rangeImage->width/2.0) / f;
			//double Y = (j - (double)rangeImage->height/2.0) / f;

			///// Get the measured range value and the estimated range value
			//double R = cvGetReal2D(rangeImage, j, i);
			//double estRange = k * R * R + l * R + m ;
			//double rConst = sqrt( 1 + X * X + Y * Y );

			///// Get the z-value by dividing thorugh the square root term
			//double z = estRange/rConst;
			//
			///// Get x and y
			//double x = X * z;
			//double y = Y * z;
			ForwardTrafoSR(i, j, cvRound(cvGetReal2D(rangeImage, j, i)), x, y, z, f, (double)rangeImage->width/2.0, (double)rangeImage->height/2.0, k, l, m);
			///cvSet2D(xyzImage, j, i, cvScalar(x, y, z));
			I = i*xyzImage->nChannels;
			((float*)(xyzImage->imageData + J))[I] = (float)x;
			((float*)(xyzImage->imageData + J))[I+1] = (float)y;
			((float*)(xyzImage->imageData + J))[I+2] = (float)z;
		}
	}	
}

void ConvertIntTo3x8Bit(short x, unsigned char& low, unsigned char& high, unsigned char& sign)
{
	if(x<0)
	{
		sign = 0;
	}
	else sign = 255;
	short abs_x = abs(x);
	if(abs_x>32767) abs_x=32767;
	low = (unsigned int) abs_x;
	high = (unsigned int)(abs_x >> 8); 
}

void Convert3x8BitToInt(unsigned char low, unsigned char high, unsigned char sign, short& x)
{
	short X = (short)low;
	X = X|(short)(high << 8);
	if(sign==0) X = -X;
	x=X;
}

/*
void Convert32FltTo3x8Bit(IplImage* fltImage, IplImage* eightBit3ChannelsImage)
{
	for(int j=0; j<fltImage->height; j++)
	{
		for(int i=0; i<fltImage->width; i++)
		{
			short int x = (short int)cvRound(cvGet2D(fltImage, j, i));
			BYTE l, h, s;
			ConvertIntTo3x8Bit(x, l, h, s);
			cvSet2D(eightBit3ChannelsImage, j, i, cvScalar(l, h, s);
		}
	}
}

void Convert3x8BitTo32Flt(IplImage* eightBit3ChannelsImage, IplImage* fltImage)
{
	for(int j=0; j<eightBit3ChannelsImage->height; j++)
	{
		for(int i=0; i<eightBit3ChannelsImage->width; i++)
		{
			CvScalar v = cvGet2D(eightBit3ChannelsImage, j, i);
			short int x=0;
			Convert3xBBitToInt(v.val[0], v.val[1], v.val[2], x); 
			cvSet2D(fltImage, j, i, x);
		}
	}
}
*/
/*
void GetZCoeffsMapUndistorted(CvMat* zCoeffs, const CvMat* A)
{
	double a[9], fx, fy, cx, cy;
	CvMat _a = cvMat(3, 3, CV_64F, a);
	cvConvert(A, &_a );

	fx = a[0]; fy = a[4];
	cx = a[2]; cy = a[5];

	for(int j=0; j<zCoeffs->rows; j++)
	{
		for(int i=0; i<zCoeffs->cols; i++)
		{
			double xd = (i - cx)/fx;
			double yd = (j - cy)/fy;
			double d = sqrt(1+xd*xd+yd*yd);
			cvSetReal2D(zCoeffs, j, i, d);
		}
	}
} 

void GetCalibratedXYZImageFromUndistortedRangeImage(IplImage* rangeImg, CvMat* zCoeffs, const CvMat* A, const CvMat* distCoeffs)
{
	for(int j=0; j<zCoeffs->rows; j++)
	{
		for(int i=0; i<zCoeffs->cols; i++)
		{
			double z = cvGetReal2D(rangeImg, j, i)/cvGetReal2D(zCoeffs, j, i);
			double x, y;
			PerspectiveProjectionInverseUndistorted(i, j, z, x, y, A);
		}
	}
}
*/

void ConvertFrame(const Mat3d& R, const Vec3d& t, Frame& F)
{
	Vec3d ex, ey, exr, eyr;
	ex.x = 1; ex.y = 0; ex.z = 0; 
	ey.x = 0; ey.y = 1; ey.z = 0; 
	Math3d::MulMatVec(R, ex, exr);
	Math3d::MulMatVec(R, ey, eyr);
	F.MakeFrameOriginDirectionsXY(Point3Dbl(t.x, t.y, t.z), Point3Dbl(exr.x, exr.y, exr.z), Point3Dbl(eyr.x, eyr.y, eyr.z));
}

void ConvertFrameBack(const ipa_utils::Frame& F, Mat3d& R, Vec3d& t)
{

	/// Get translation vector
	t.x = (float)F[0];
	t.y = (float)F[1];
	t.z = (float)F[2];

	/// Get rotation (Rodriguez) vector 
	Point3Dbl rod = Point3Dbl(1.0, F[4], F[5]);
	rod.ToCart();
	rod.ScalarMul(F[3]);
	CvMat* _r = cvCreateMat(1, 3, CV_32FC1);
	cvSetReal1D(_r, 0, rod.m_x);
	cvSetReal1D(_r, 1, rod.m_y);
	cvSetReal1D(_r, 2, rod.m_z);

	/// Apply Rodrigues conversion
	CvMat* _R = cvCreateMat(3, 3, CV_32FC1);
	cvRodrigues2(_r, _R);
	
	/// Set rotation matrix components
	R.r1 = (float)cvGetReal2D(_R, 0, 0);
	R.r2 = (float)cvGetReal2D(_R, 0, 1);
	R.r3 = (float)cvGetReal2D(_R, 0, 2);
	R.r4 = (float)cvGetReal2D(_R, 1, 0);
	R.r5 = (float)cvGetReal2D(_R, 1, 1);
	R.r6 = (float)cvGetReal2D(_R, 1, 2);
	R.r7 = (float)cvGetReal2D(_R, 2, 0);
	R.r8 = (float)cvGetReal2D(_R, 2, 1);
	R.r9 = (float)cvGetReal2D(_R, 2, 2);
}

} // end namespace ipa_utils

