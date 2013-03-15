#include "object_categorization/BlobFeature.h"

using namespace ipa_utils;

BlobFeature::BlobFeature(int Id, int x, int y, int r, double Res, /*double Avg, double Mag,*/ double Phi, int Label)
{ 
	m_Id=Id; 
	m_x=x;
	m_y=y;
	m_r=r;
	m_Res=Res; 
	//m_Avg=Avg; 
	//m_Mag=Mag;
	m_Phi=Phi;
	//m_Label = Label;
}


std::string BlobFeature::Str() const
{
	std::stringstream s;
	s << m_Id << " " << m_x << " " << m_y << " " << m_r << " " << m_Res << /*" " << m_Avg << " " << m_Mag <<*/ " " << m_Phi << "\n"; // " " << m_Label << "\n";
	(*this).m_D.Str();
	//s << m_D_String;
	return s.str();
}


void BlobFeature::DrawInIplImage(IplImage* Src, int Mode, bool UseKeyColor)
{
	int R=0, G=0, B=255;
	if(UseKeyColor)
	{
		int I=(int)m_D.Sum();
		IntToCol(I, R, G, B);
		//HueToCol((int) m_D[0], 16, R, G, B);
	}
	CvScalar Col=CV_RGB(R, G, B);

	if(Mode==0) /// Mode==0: draw a circle or rectangle (using P.m_z as radius) and a line from center to the circle boundary (using m_Phi).
	{
		//cvRectangle(Src, cvPoint(m_x-m_r, m_y-m_r), cvPoint(m_x+m_r, m_y+m_r), Col, 1); 
		cvCircle(Src, cvPoint(m_x, m_y), m_r, Col, 1);
		int dx=cvRound(m_r*cos(m_Phi));
		int dy=cvRound(m_r*sin(m_Phi));
		cvLine(Src, cvPoint(m_x, m_y), cvPoint(cvRound((double)m_x+dx), cvRound((double)m_y+dy)), Col, 1);
	}
	
	if(Mode==1) /// Mode==1: draw points
	{
		cvSet2D(Src, m_y, m_x, Col);
	}
}
