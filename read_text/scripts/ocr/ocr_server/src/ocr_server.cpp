/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include <algorithm>
#include <string>
#include <fstream>
#include <cv.h>
#include <highgui.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include "ros/ros.h"
#include "ocr_server/TextRecognition.h"

using namespace std;
using namespace sensor_msgs::image_encodings;
using namespace ocr_server;

class OcrServerNode {
public:
  vector<string> first_names_;
  vector<string> last_names_;
  vector<string> spam_filter_;

  int max_distance_;
  string tmp_folder_;
  ros::ServiceServer ocr_srv_;
  sensor_msgs::CvBridge bridge_;

  OcrServerNode() {
    ros::NodeHandle private_nh("~");
    private_nh.param(string("temp_folder"), tmp_folder_, string("/tmp"));
    private_nh.param(string("max_distance"), max_distance_, 3);
    ocr_srv_ = private_nh.advertiseService("ocr", &OcrServerNode::recognizeCB, this);
  }

  virtual ~OcrServerNode() {
  }

  bool recognize(const sensor_msgs::Image& image, vector<string>& lines) {
    string tmp_image_file = tmp_folder_ + "/" + "ocr_temp_img.png";
    string tmp_results_file = tmp_image_file + ".txt";

    IplImage *cv_image = NULL;
    try {
      if (bridge_.fromImage(image, "bgr8")) {
        cv_image = bridge_.toIpl();
      }
      else {
        ROS_ERROR("Unable to convert %s image to bgr8", image.encoding.c_str());
        return false;
      }
    }
    catch(...) {
      ROS_ERROR("Unable to convert %s image to ipl format", image.encoding.c_str());
      return false;
    }

    cv::Mat img = cv_image;
    cv::Mat scaled_img;
    double scale = 1.5;
    cv::resize(img, scaled_img, cv::Size(0,0), scale, scale, cv::INTER_LANCZOS4);
    cv::Mat gray_img;
    cv::cvtColor(scaled_img, gray_img, CV_BGRA2GRAY);
    cv::Mat norm_img;
    cv::normalize(gray_img, norm_img, 0, 255, CV_MINMAX);

    ROS_INFO("Saving image into temporary file %s",tmp_image_file.c_str());
    try {
      cv::imwrite(tmp_image_file.c_str(), norm_img);
    }
    catch(...) {
      ROS_ERROR("Unable to save temporary image");
      return false;
    }

    ROS_INFO("Running tesseract on image.");
    string cmd = "$(rospack find tesseract)/bin/tesseract " + tmp_image_file + " " + tmp_image_file;
    int32_t sys_ret = system(cmd.c_str());

    ROS_INFO("Removing temporary image file.");
    cmd = "rm " + tmp_image_file;
    sys_ret = system(cmd.c_str());

    ROS_INFO("Reading in tesseract results.");
    string line;
    ifstream textstream (tmp_results_file.c_str());
    while (getline(textstream, line)) {
      lines.push_back(line);
    }
    textstream.close();
    ROS_INFO("Removing results file.");
    cmd = "rm " + tmp_results_file;
    sys_ret = system(cmd.c_str());

    return true;
  }

  int levenshteinWordDistance(string s, string t)
  {
    int n = s.length();
    int m = t.length();

    if (n == 0) return m;
    if (m == 0) return n;

    vector<vector<int> > d;
    d.resize(n+1);
    for(int i=0;i<n+1;i++) {
      d[i].resize(m + 1);
    }

    for ( int i = 0; i <= n; d[i][0] = i++ );
    for ( int j = 1; j <= m; d[0][j] = j++ );

    for ( int i = 1; i <= n; i++ ) {
      char sc = s.at( i-1 );
      for (int j = 1; j <= m;j++) {
        int v = d[i-1][j-1];
        if ( t.at( j-1 ) !=  sc ) v++;
        d[i][j] = min( min( d[i-1][ j] + 1, d[i][j-1] + 1 ), v );
      }
    }
    return d[n][m];
  }

  vector<string> tokenize(const string& str,const string& delimiters)
      {
    vector<string> tokens;
    string::size_type delimPos = 0, tokenPos = 0, pos = 0;

    if(str.length()<1)  return tokens;
    while(1){
      delimPos = str.find_first_of(delimiters, pos);
      tokenPos = str.find_first_not_of(delimiters, pos);

      if(string::npos != delimPos){
        if(string::npos != tokenPos){
          if(tokenPos<delimPos){
            tokens.push_back(str.substr(pos,delimPos-pos));
          }else{
            tokens.push_back("");
          }
        }else{
          tokens.push_back("");
        }
        pos = delimPos+1;
      } else {
        if(string::npos != tokenPos){
          tokens.push_back(str.substr(pos));
        } else {
          tokens.push_back("");
        }
        break;
      }
    }
    return tokens;
      }

  string& trim(string& str)
  {
    // Trim Both leading and trailing spaces
    size_t startpos = str.find_first_not_of(" \t"); // Find the first character position after excluding leading blank spaces
    size_t endpos = str.find_last_not_of(" \t"); // Find the first character position from reverse af
    // if all spaces or empty return an empty string
    if(( string::npos == startpos ) || ( string::npos == endpos))
      str = "";
    else
      str = str.substr(startpos, endpos-startpos+1);
    return str;
  }

  bool lookupDictionary(vector<string> lines, string& entry)
  {

    int min_distance = 2*max_distance_ + 1;
    int min_name = 0;

    for (size_t k=0; k < spam_filter_.size(); k++) {
      for (size_t j=0; j<lines.size(); j++) {
        string line = lines[j];
        vector<string> words = tokenize(line," ");

        for (size_t i=0; i<words.size(); i++)
        {
          string word = trim(words[i]);
          if(word.length()==0) continue;
          transform(word.begin(), word.end(), word.begin(), ::tolower);

          int distance = levenshteinWordDistance(word, spam_filter_[k]);
          if (distance == 0) {
            entry = "junk";
            return true;
          }
        }
      }
    }

    for (size_t k=0; k < first_names_.size(); k++) {

      int first_distance = max_distance_;
      int last_distance = max_distance_;

      for (size_t j=0; j<lines.size(); j++) {
        string line = lines[j];
        vector<string> words = tokenize(line," ");

        for (size_t i=0; i<words.size(); i++)
        {
          string word = trim(words[i]);
          if(word.length()==0) continue;
          transform(word.begin(), word.end(), word.begin(), ::tolower);

          int distance = std::min(levenshteinWordDistance(word, first_names_[k]), max_distance_);
          if (distance < first_distance)
            first_distance = distance;

          distance = std::min(levenshteinWordDistance(word, last_names_[k]), max_distance_);
          if (distance < last_distance)
            last_distance = distance;
        }
      }

      int dist = first_distance + last_distance;
      if (dist < min_distance) {
        min_distance = dist;
        min_name = k;
      }

    }

    entry = last_names_[min_name];
    return (min_distance < 2*max_distance_);
  }

  bool recognizeCB(TextRecognition::Request &req, TextRecognition::Response &res )
  {

    ROS_INFO("ocr request received");
    vector<string> lines;
    if(!recognize(req.image, lines)) {
      ROS_ERROR("Error occurred during OCR");
      return false;
    }
    ROS_INFO("recognized text (%i lines):\n", (int)lines.size());
    for (size_t i=0; i<lines.size(); i++) {
      printf("%i. %s\n", (int)i, lines[i].c_str());
    }
    res.lines = lines;

    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ocr_server");
  OcrServerNode ocr;

  ROS_INFO("Ready to run ocr on images.");
  ros::spin();

  return 0;
}
