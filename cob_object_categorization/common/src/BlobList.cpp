#include "object_categorization/BlobList.h"

using namespace ipa_utils;


std::string BlobList::Str()
{
	std::stringstream s;
	BlobList::iterator It;
	for(It=this->begin(); It!=this->end(); It++)
		s << It->Str();
	return s.str();
}


void BlobList::DrawListInIplImage(IplImage* Src, int Mode, bool UseKeyColor)
{
	BlobList::iterator It;
	for(It=this->begin(); It!=this->end(); It++)
		It->DrawInIplImage(Src, Mode, UseKeyColor);
}


int BlobList::Load(std::string Name, bool Append)
{
	if(!Append) clear();
	std::ifstream f(Name.c_str(), std::fstream::in);
	if(!f.is_open())
	{
		std::cout << "BlobList::Load: Could not load '" << Name << "'" << std::endl;
		return RET_FAILED;
	}
	int Size=0, Dim=0; 
	f >> Size >> Dim;
	for(int i=0; i<Size; i++)
	{
		BlobFeature fp;
		//f >> fp.m_y >> fp.m_x >> fp.m_r >> fp.m_Phi >> fp.m_Label >> fp.m_Id;
		f >> fp.m_y >> fp.m_x >> fp.m_r >> fp.m_Phi >> fp.m_Id;
		for(int j=0; j<Dim; j++)
		{
			float val;
			f >> val;
			fp.m_D.push_back(val);
		}
		this->push_back(fp);
	}
	f.close();
	return RET_OK;
}

int BlobList::Load(std::ifstream* File, bool Append)
{
	if(!Append) clear();

	int Size=0, Dim=0, FrameDim=0; 
	*File >> Size >> Dim >> FrameDim;
	for(int i=0; i<Size; i++)
	{
		BlobFeature fp;
		//f >> fp.m_y >> fp.m_x >> fp.m_r >> fp.m_Phi >> fp.m_Label >> fp.m_Id;
		*File >> fp.m_y >> fp.m_x >> fp.m_r >> fp.m_Phi >> fp.m_Id;
		for(int j=0; j<Dim; j++)
		{
			float val;
			*File >> val;
			fp.m_D.push_back(val);
		}
		fp.m_Frame.clear();
		for(int j=0; j<FrameDim; j++)
		{
			double val;
			*File >> val;
			fp.m_Frame.push_back(val);
		}
		this->push_back(fp);
	}

	return RET_OK;
}


int BlobList::Save(std::string Name)
{
	std::ofstream f(Name.c_str(), std::fstream::out);
	if(!f.is_open())
	{
		std::cout << "BlobList::Save: Could not open '" << Name << "'" << std::endl;
		return RET_FAILED;
	}
	if(this->size()==0)
	{
		std::cout << "BlobList::Save: No features to be saved for '" << Name << "'" << std::endl;
		return RET_FAILED;
	}

	int Dim = (int)(this->begin()->m_D.size());
	f << this->size() << " " << Dim << std::endl;
	BlobList::iterator It;
	for(It=this->begin(); It!=this->end(); It++)
	{	
		//f << It->m_y << " " << It->m_x << " " << It->m_r << " " << It->m_Phi << " " << It->m_Label << " " << It->m_Id << "\n";
		f << It->m_y << " " << It->m_x << " " << It->m_r << " " << It->m_Phi << " " << It->m_Id << "\n";
		for(int j=0; j<Dim; j++)
			f << " " << It->m_D[j];
		f << "\n";
	}
	f.close();
	
	return RET_OK;
}


int BlobList::Save(std::ofstream* File)		// /�nderung RiB/
{
	if(this->size()==0)
	{
		*File << this->size() << " " << 0 << " " << 0 << std::endl;
		std::cout << "BlobList::Save: No features to be saved." << std::endl;
		return RET_FAILED;
	}

	int Dim = (int)(this->begin()->m_D.size());
	int FrameDim = (this->begin())->m_Frame.size();
	*File << this->size() << " " << Dim << " " << FrameDim << std::endl;

	BlobList::iterator It;
	for(It=this->begin(); It!=this->end(); It++)
	{	
		//f << It->m_y << " " << It->m_x << " " << It->m_r << " " << It->m_Phi << " " << It->m_Label << " " << It->m_Id << "\n";
		*File << It->m_y << " " << It->m_x << " " << It->m_r << " " << It->m_Phi << " " << It->m_Id << "\n";
		for(int j=0; j<Dim; j++)
			*File << " " << It->m_D[j];
		*File << "\n";
		for(int j=0; j<FrameDim; j++) *File << " " << It->m_Frame[j];
		*File << "\n";
	}
	
	return RET_OK;
}


void BlobList::DeleteBorderPoints(IplImage* ImageMask, int MaskVal)
{
	BlobList::iterator It=begin();
	CvScalar Col;
	for(;It!=end();)
	{
		bool Flag=false;
		
		/// Search top/bottom rows
		for(int i=-It->m_r; i<=It->m_r; i++)
		{
			// top
			Col = cvGet2D(ImageMask, It->m_y-It->m_r, It->m_x+i);
			if(Col.val[0]+Col.val[1]+Col.val[2]<=MaskVal)
			{	
				Flag=true;
				goto Delete;
			}

			// bottom
			Col = cvGet2D(ImageMask, It->m_y+It->m_r, It->m_x+i);
			if(Col.val[0]+Col.val[1]+Col.val[2]<=MaskVal)
			{	
				Flag=true;
				goto Delete;
			}
		}

		/// Search left/right colums
		for(int j=-It->m_r+1; j<It->m_r; j++)
		{
			// left
			Col = cvGet2D(ImageMask, It->m_y+j, It->m_x-It->m_r);
			if(Col.val[0]+Col.val[1]+Col.val[2]<=MaskVal)
			{	
				Flag=true;
				goto Delete;
			}

			// right
			Col = cvGet2D(ImageMask, It->m_y+j, It->m_x+It->m_r);
			if(Col.val[0]+Col.val[1]+Col.val[2]<=MaskVal)
			{	
				Flag=true;
				goto Delete;
			}
		}
	
		Delete:
		if(Flag)
		{
			It=erase(It);
		}
		else
			It++;
	}
}


void BlobList::FilterScaleInterval(int ScaleMin, int ScaleMax)
{
	BlobList::iterator It;
	for(It=this->begin(); It!=this->end();)
	{
		if(It->m_r<ScaleMin || It->m_r>ScaleMax)
		{
			It=erase(It);
		}
		else
		{
			It++;
		}
	}
}


void BlobList::FilterThreshold(int Threshold, bool SmallerThan)
{
	BlobList::iterator It;
	for(It=this->begin(); It!=this->end();)
	{
		if(SmallerThan)
		{
			if(It->m_Res>Threshold)
				It=erase(It);
			else
				It++;
		}
		else
		{
			if(It->m_Res<Threshold)
				It=erase(It);
			else
				It++;
		}
	}
}
