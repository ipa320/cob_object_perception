/*
 *      Transforms xml file from ICDAR competition xml-style to xml-style used by read_evaluation
 *
 *
 *
 */


#include <iostream>
#include <fstream>
#include <dirent.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char* argv[])
{
  std::ifstream inn;
  std::string str;
  DIR *pDIR;
  struct dirent *entry;
  unsigned x, y, width, height;
  std::string word;

  std::string newFileName = "/home/rmb-rh/script/train.xml";
  std::ofstream newFile;
  newFile.open(newFileName.c_str());
  newFile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
  newFile << std::endl;
  newFile << "<tagset>";
  newFile << std::endl;

  if ((pDIR = opendir("/home/rmb-rh/script/all")))
  {
    while ((entry = readdir(pDIR)))
    {
      if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
      {
        str = "/home/rmb-rh/script/all/";
        str.append(entry->d_name);
        inn.open(str.c_str());
        if (inn.is_open())
        {
          std::string name;
          name=entry->d_name;
          name=name.substr(name.find_first_of('_')+1,name.find_first_of('.')-name.find_first_of('_'));
          name.append("jpg");
          newFile << "<image>" << std::endl << " <imageName>" << name << "</imageName>" << std::endl
              << "<taggedRectangles>" << std::endl;
              
          // e.g.: <taggedRectangle x="199" y="531" width="160" height="44" modelType="1" text="FREEDOM" />
          while (getline(inn, str))
          {
            std::string remainingString = str;
            x = atoi(remainingString.substr(0, remainingString.find_first_of(',')).c_str());
            remainingString = remainingString.substr(remainingString.find_first_of(',') + 1, remainingString.length()
                - remainingString.find_first_of(','));
            y = atoi(remainingString.substr(0, remainingString.find_first_of(',')).c_str());
            remainingString = remainingString.substr(remainingString.find_first_of(',') + 1, remainingString.length()
                - remainingString.find_first_of(','));
            width = atoi(remainingString.substr(0, remainingString.find_first_of(',')).c_str()) - x;
            remainingString = remainingString.substr(remainingString.find_first_of(',') + 1, remainingString.length()
                - remainingString.find_first_of(','));
            height = atoi(remainingString.substr(0, remainingString.find_first_of(',')).c_str()) - y;
            remainingString = remainingString.substr(remainingString.find_first_of(',') + 1, remainingString.length()
                - remainingString.find_first_of(','));
            word = remainingString.substr(0, remainingString.find_first_of('\n'));
            newFile << "<taggedRectangle x=\"" << x << "\" y=\"" << y << "\" width=\"" << width << "\" height=\""
                << height << "\" modelType=\"1\" text=" << word << " />" << std::endl;
          }
          newFile << "</taggedRectangles>" << std::endl << "</image>" << std::endl;
        }

      }
      inn.close();
    }
    closedir(pDIR);
  }
  newFile << "</tagset>";

  return 0;
}
