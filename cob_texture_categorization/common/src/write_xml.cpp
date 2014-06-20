
/*
 *      Transforms matlab generated textfile of manual texture categorization data in to xml file
 *
 *
 *
 */

#include <cob_texture_categorization/write_xml.h>
#include <cob_texture_categorization/texture_features.h>


#include <fstream>
#include <dirent.h>

#include <stdio.h>
#include <stdlib.h>

#include <sstream>







xml_write::xml_write()
{
}

std::string xml_write::write_dataset(struct data *daten, int status, struct feature_results *results)
{
	std::string dataset;
	if(status == 1)
	{
		dataset = "  <class name=\"" + (*daten).data_in[4] + "\">\n" + "    <object type=\"" + (*daten).data_in[3] + "\">\n";

	}else if(status == 2)
	{
		dataset = "    <object type=\"" + (*daten).data_in[3] + "\">\n";
	}
	dataset = dataset + "      <image number=\"" + (*daten).data_in[2] + "\">\n" +"        <name value=\"" + (*daten).data_in[1] + "\"/>\n" +  "        <path value=\"" + (*daten).data_in[0] + "\"/>\n";


//		dataset = dataset + "        <plain-colored value=\" 0 \"/>\n" ;
//		dataset = dataset + "        <bi-colored value=\" 0 \"/>\n" ;
//		dataset = dataset + "        <colorfulness value=\"" + (*results).colorfulness + "\"/>\n" ;
//		dataset = dataset + "        <dominant-color value=\"" + (*results).dom_color + "\"/>\n" ;
//		dataset = dataset + "        <secondary-dominant-color value=\"" + (*results).dom_color2 + "\"/>\n" ;
//		dataset = dataset + "        <value-lightness-brightness value=\"" + (*results).v_mean + "\"/>\n" ;
//		dataset = dataset + "        <variety-of-value-lightness-brightness value=\"" + (*results).v_std + "\"/>\n" ;
//		dataset = dataset + "        <saturation value=\"" + (*results).s_mean + "\"/>\n" ;
//		dataset = dataset + "        <variety-of-saturation value=\"" + (*results).s_std + "\"/>\n" ;
//		dataset = dataset + "        <average-primitive--texel-texton--size value=\"" + (*results).avg_size + "\"/>\n" ;
//		dataset = dataset + "        <number-of-primitives--texels-textons value=\"" + (*results).prim_num + "\"/>\n" ;
//		dataset = dataset + "        <strength-of-primitives--texels-textons value=\"" + (*results).prim_strength + "\"/>\n" ;
//		dataset = dataset + "        <regularity-of-primitives--texels-textons value=\"" + (*results).prim_regularity + "\"/>\n" ;
//		dataset = dataset + "        <contrast value=\"" + (*results).contrast + "\"/>\n" ;
//		dataset = dataset + "        <line-likeness value=\"" + (*results).line_likeness + "\"/>\n" ;
//		dataset = dataset + "        <THREED-roughness value=\" 0 \"/>\n" ;
//		dataset = dataset + "        <directionality-regularity value=\"" + (*results).direct_reg + "\"/>\n" ;
//		dataset = dataset + "        <lined value=\"" + (*results).lined + "\"/>\n" ;
//		dataset = dataset + "        <horizontality value=\" 0 \"/>\n" ;
//		dataset = dataset + "        <checked value=\"" + (*results).checked + "\"/>\n" ;


			dataset = dataset + "        <plain-colored value=\" 0 \"/>\n" ;
			dataset = dataset + "        <bi-colored value=\" 0 \"/>\n" ;
			std::ostringstream Str;
			Str << (*results).colorfulness;
			std::string str_val(Str.str());
			dataset = dataset + "        <colorfulness value=\"" + str_val + "\"/>\n" ;
			std::ostringstream Str1;
			Str1 << (*results).dom_color;
			std::string str_val1(Str1.str());
			dataset = dataset + "        <dominant-color value=\"" + str_val1 + "\"/>\n" ;
			std::ostringstream Str2;
			Str2 << (*results).dom_color2;
			std::string str_val2(Str2.str());
			dataset = dataset + "        <secondary-dominant-color value=\"" +str_val2  + "\"/>\n" ;
			std::ostringstream Str3;
			Str3 << (*results).v_mean;
			std::string str_val3(Str3.str());
			dataset = dataset + "        <value-lightness-brightness value=\"" +str_val3  + "\"/>\n" ;
			std::ostringstream Str4;
			Str4 << (*results).v_std;
			std::string str_val4(Str4.str());
			dataset = dataset + "        <variety-of-value-lightness-brightness value=\"" +str_val4  + "\"/>\n" ;
			std::ostringstream Str5;
			Str5 << (*results).s_mean;
			std::string str_val5(Str5.str());
			dataset = dataset + "        <saturation value=\"" +str_val5  + "\"/>\n" ;
			std::ostringstream Str6;
			Str6 << (*results).s_std;
			std::string str_val6(Str6.str());
			dataset = dataset + "        <variety-of-saturation value=\"" +str_val6  + "\"/>\n" ;
			std::ostringstream Str7;
			Str7 << (*results).avg_size;
			std::string str_val7(Str7.str());
			dataset = dataset + "        <average-primitive--texel-texton--size value=\"" + str_val7 + "\"/>\n" ;
			std::ostringstream Str8;
			Str8 << (*results).prim_num;
			std::string str_val8(Str8.str());
			dataset = dataset + "        <number-of-primitives--texels-textons value=\"" + str_val8 + "\"/>\n" ;
			std::ostringstream Str9;
			Str9 << (*results).prim_strength;
			std::string str_val9(Str9.str());
			dataset = dataset + "        <strength-of-primitives--texels-textons value=\"" + str_val9 + "\"/>\n" ;
			std::ostringstream Str10;
			Str10 << (*results).prim_regularity;
			std::string str_val10(Str10.str());
			dataset = dataset + "        <regularity-of-primitives--texels-textons value=\"" + str_val10 + "\"/>\n" ;
			std::ostringstream Str11;
			Str11 << (*results).contrast;
			std::string str_val11(Str11.str());
			dataset = dataset + "        <contrast value=\"" + str_val11 + "\"/>\n" ;
			std::ostringstream Str12;
			Str12 << (*results).line_likeness;
			std::string str_val12(Str12.str());
			dataset = dataset + "        <line-likeness value=\"" + str_val12 + "\"/>\n" ;
			dataset = dataset + "        <THREED-roughness value=\" 0 \"/>\n" ;
			std::ostringstream Str13;
			Str13 << (*results).direct_reg;
			std::string str_val13(Str13.str());
			dataset = dataset + "        <directionality-regularity value=\"" + str_val13 + "\"/>\n" ;
			std::ostringstream Str14;
			Str14 << (*results).lined;
			std::string str_val14(Str14.str());
			dataset = dataset + "        <lined value=\"" + str_val14 + "\"/>\n" ;
			dataset = dataset + "        <horizontality value=\" 0 \"/>\n" ;
			std::ostringstream Str15;
			Str15 << (*results).checked;
			std::string str_val15(Str15.str());
			dataset = dataset + "        <checked value=\"" + str_val15 + "\"/>\n" ;


	if(status == 1)
	{
		dataset = dataset + "      </image>\n" +"    </object>\n"+"  </class>\n";
	}else if(status == 2)
	{
		dataset = dataset + "      </image>\n" +"    </object>\n";
	}else if(status == 3)
	{
		dataset = dataset + "      </image>\n";
	}



	return dataset;
}

void xml_write::write_data(struct stat *status, std::string path, struct data *daten, struct feature_results *results)
{
	std::ofstream inn(path.c_str(), std::ofstream::in );
	std::ifstream newFile( path.c_str(), std::ifstream::out);
	std::string newContent;
	std::string str;
	std::string search;
	FILE* filehandle;

	  if((filehandle = fopen("/home/rmb-dh/XML/output/db.xml", "r")) == NULL)			//Creates new xml file if non exists
	  {
		 std::ofstream inn("/home/rmb-dh/XML/output/db.xml", std::ofstream::out );
		 inn << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
		 inn << "<texture_categorization_db>\n";
		 inn << "</texture_categorization_db>";
		 inn << std::endl;
		 inn.close();
	  }

  while(!newFile.eof())
		  {


	  	  	getline(newFile, str);
	  	  	if((*status).create == 1)									//create new class, object and image
	  	  	{

  				  while(strstr(str.c_str(), "</class>") == NULL)
  				  {
			  	  	  newContent = newContent + str + "\n";
		  	  		  getline(newFile, str);
	  	  		  	  if(strstr(str.c_str(), "</texture_categorization_db>") != NULL)
	  	  		  	  {
	  	  		  		  	  str = write_dataset(daten, (*status).create, results);
	  	  		  		  	  newContent = newContent + str + "</texture_categorization_db>";
	  	  		  		  	  newFile.close();
	  	  		  		  	  inn << newContent;
	  	  		  		  	  inn.close();
	  	  		  		  	  return;
	  	  		  	  }
  				  }
  				  newContent = newContent + str + "\n";
  				str = write_dataset(daten, (*status).create, results);
  	  			newContent = newContent + str;
  	  			while(!newFile.eof())
				{
  	  				getline(newFile, str);
  	  				newContent = newContent + str + "\n";
				}
	  	  	}else if((*status).create == 2)									//create new object and image
	  	  	{

  	  		  	  search = "<class name=\"" + (*daten).data_in[4] + "\">";
  	  		  	  if(strstr(str.c_str(), search.c_str()) != NULL)
  	  		  	  {
  	  		  		  newContent = newContent + str + "\n";
  	  		  		  while(!newFile.eof())
  	  		  			  {
	  		  		 	 	 if((*status).upper != "")
	  		  		 	 	 {
	  		  		 	 		 getline(newFile, str);
	  		  		 	 		 newContent = newContent + str + "\n";
  	  		  		 	 	 	 search = "<object type=\"" + (*status).upper + "\">";
  	  		  		 	 	 	 while(strstr(str.c_str(), search.c_str()) == NULL && (*status).upper != "")
  	  		  		 	 	 	 {
  	  		  		 	 	 		 getline(newFile, str);
  	  		  		 	 	 		 newContent = newContent + str + "\n";
  	  		  		 	 	 	 }
  	  		  		 	 	 	 while(strstr(str.c_str(), "</object>") == NULL)
  	  		  		 	 	 	 {
  	  		  		 	 	 		 getline(newFile, str);
  	  		  		 	 	 		 newContent = newContent + str + "\n";
  	  		  		 	 	 	 }
  	  		  		 	 	 }
  	  		  		 	 	 str = write_dataset(daten, (*status).create, results);
  	  		  		 	 	 newContent = newContent + str;
  	  		  		 	 	 while(!newFile.eof())
  	  		  		 	 	 {
  	  		  		 	 		 getline(newFile, str);
  	  		  		 	 		 newContent = newContent + str + "\n";
  	  		  		 	 	 }
  	  		  			  }
  	  		  	  }else
  	  		  	  {
  	  		  		  newContent = newContent + str + "\n";
  	  		  	  }

	  	  	  }else if((*status).create == 3)								//create new image
	  	  	  {

  	 	  	  	  search = "<class name=\"" + (*daten).data_in[4] + "\">";
  	 	  	  	  while(strstr(str.c_str(), search.c_str()) == NULL)
  	 	  	  	  {
  		  	  		  newContent = newContent + str + "\n";
  		  	  		  getline(newFile, str);
  	 	  	  	  }
  	 	  	  	  newContent = newContent + str + "\n";
  	 	  	  	  while(!newFile.eof())
  	 	  	  	  {

  	 	  	  		  getline(newFile, str);
  	 	  	  		  newContent = newContent + str + "\n";
  	 	  	  		  search = "<object type=\"" + (*daten).data_in[3] + "\">";
  	 	  	  		  if(strstr(str.c_str(), search.c_str()) != NULL)
  	 	  	  		  {

  	 	  	  			  if((*status).upper != "")
  	 	  	  			  {
  	 	  	  					  search = "<image number=\"" + (*status).upper + "\">";
  	 	  	  					  while(strstr(str.c_str(), search.c_str()) == NULL)
  	 	  	  					  {
  	 	  	  						  getline(newFile, str);
  	 	  	  						  newContent = newContent + str + "\n";
  	 	  	  					  }
  	 	  	  					  while(strstr(str.c_str(), "</image>") == NULL)
  	 	  	  					  {
  	 	  	  						  getline(newFile, str);
  	 	  	  						  newContent = newContent + str + "\n";
  	 	  	  					  }
  	 	  	  			  }
  	 	  	  			  str = write_dataset(daten, (*status).create, results);
  	 	  	  			  newContent = newContent + str;
  	 	  	  			  while(!newFile.eof())
  	 	  	  			  {
  	 	  	  				  getline(newFile, str);
  	 	  	  				  newContent = newContent + str + "\n";
  	 	  	  			  }
  	 	  	  		  }
  	 	  	  	  }





	  	  	  }
		  }

  inn << newContent;
  newFile.close();
  inn.close();

}




void xml_write::getstat(std::string path, struct data *daten, struct stat *status)
{

	std::ifstream searchFile(path.c_str(), std::fstream::out);
	std::string search;
	std::string str;
	int compa;
	int compb = 0;
	(*status).create = 0;
	(*status).upper = "";



	while(!searchFile.eof())
	{
		  	  getline(searchFile, str);
	  	  	  search = "<class name=\"" + (*daten).data_in[4] + "\">";
		  	  if(strstr(str.c_str(), search.c_str()) != NULL)
		  	  {
		  		  while(strstr(str.c_str(), "</class>") == NULL)
		  		  {
		  			  getline(searchFile, str);
		  			  search = "<object type=\"" + (*daten).data_in[3] + "\">";
		  			  if(strstr(str.c_str(), search.c_str()) != NULL)
		  			  {
		  				  (*status).upper = "";

		  				  while(strstr(str.c_str(), "</object>") == NULL)
		  				  {
							  getline(searchFile, str);
							  search = "<image number=\"" + (*daten).data_in[2] + "\">";
							  if(strstr(str.c_str(), search.c_str()) != NULL)
							  {
								  std::cout << "Error: Image already exists:" + (*daten).data_in[1] + "\n";
								  (*status).create = 4;
								  return;
							  }
							  search = "<image number=\"";
							  if(strstr(str.c_str(), search.c_str()) != NULL)
							  {
								  if((*status).create <= 2) (*status).create = 3;		//Create Image

								  std::string test2 = str.substr(str.find_first_of('=')+2, 2);
								  std::string test = (str.substr(21,2)).c_str();
								  compb = atoi((str.substr(21,2)).c_str());
								  compa  = atoi(((*daten).data_in[2]).c_str());
								  if(compb<compa) (*status).upper = test;
							  }
		  				  }

		  			  }else
		  			  {
		  				  if((*status).create <= 1) (*status).create = 2;	//Create Object
		  				  if(strstr(str.c_str(), "<object type=") != NULL)
		  				  {
							  std::string test = (str.substr(18,2)).c_str();
							  compb = atoi((str.substr(18,2)).c_str());
							  compa = atoi(((*daten).data_in[3]).c_str());
							  if(compb<compa) (*status).upper = test;
		  				  }
		  			  }
		  		  }

		  	  }else
		  	  {
		  		  if((*status).create <= 0) (*status).create = 1;  //Create Class
		  	  }
	  }
	  searchFile.close();

}







void xml_write::write_main(struct feature_results *results, struct data *daten, struct stat *status)
{


//  DIR *pDIR;
//  std::string str;
////  struct data daten;
//  struct dirent *entry;
////  struct stat status;
////  status.create = 0;
////  status.upper = "";
//
//  FILE* filehandle;
//
//  std::string path = "/home/rmb-dh/testdaten.txt";
//  std::string patho="/home/rmb-dh/XML/output/db.xml";
//
////	std::cout << "Texture Categorization Data in to XML File\n";
////	std::cout << "Please insert Destination Path:";
////	std::cin >> patho;
////	std::cout << "Please insert Source Path:";
////	std::cin >> path;
//
//
//  if((filehandle = fopen("/home/rmb-dh/XML/output/db.xml", "r")) == NULL)			//Creates new xml file if non exists
//  {
//	 std::ofstream inn("/home/rmb-dh/XML/output/db.xml", std::ofstream::out );
//	 inn << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
//	 inn << "<texture_categorization_db>\n";
//	 inn << "</texture_categorization_db>";
//	 inn << std::endl;
//	 inn.close();
//  }
//
//
//
//  if ((pDIR = opendir("/home/rmb-dh/txt")))
//  {
//    while ((entry = readdir(pDIR)))
//    {
//      if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
//      {
//        str = "/home/rmb-dh/txt/";
//        str.append(entry->d_name);
//        if(!strchr(str.c_str(), '~'))
//        {
//        std::ifstream input(str.c_str(), std::ifstream::out);
//		  getline(input, str);
//		  while(!input.eof())
//		  {
//			  if(strstr(str.c_str(), "NEW_IMAGE") != NULL)
//			  {
//				  for(int i = 0; i<=24; i++)						//Write data from inputfile into struct data
//				  {
//					  getline(input, str);
//					  if(i<5)
//					  {
//						  str.erase(str.end() - 1);				//Is needed for input files created on linux os
////						  (*daten).data_in[i] = str;
//					  }
//
//					  if(i>=5)
//					  {
//						  str.erase(str.end() - 1);
////						  (*daten).param[i-5] = str;
//					  }
//
//				  }
////				  getstat(patho, &daten, &status);					//checks where data have to be insert
////				  (*write_data)(&status, patho, &daten, &results);				//writes data into xml file
//				  getline(input, str);
//			  }else
//			  {
//				  getline(input, str);
//			  }
//
//		  }
//        }
//       }
//    }
//  }
}

