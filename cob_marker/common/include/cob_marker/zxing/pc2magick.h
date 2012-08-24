/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2012 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: TODO FILL IN PROJECT NAME HERE
* \note
* ROS stack name: cob_object_perception
* \note
* ROS package name: cob_marker
*
* \author
* Author: Joshua Hampp
*
* \date Date of creation: 24.8.2012
*
* \brief
* 6DOF marker
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/

#ifndef PC2MAGICK_H_
#define PC2MAGICK_H_

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>

#include <iostream>
#include <fstream>
#include <string>
#include <Magick++.h>
#include "cob_marker/zxing/MagickBitmapSource.h"
#include <zxing/common/Counted.h>
#include <zxing/Binarizer.h>
#include <zxing/MultiFormatReader.h>
#include <zxing/Result.h>
#include <zxing/ReaderException.h>
#include <zxing/common/GlobalHistogramBinarizer.h>
#include <zxing/common/HybridBinarizer.h>
#include <exception>
#include <zxing/Exception.h>
#include <zxing/common/IllegalArgumentException.h>
#include <zxing/BinaryBitmap.h>
#include <zxing/DecodeHints.h>

#include <zxing/qrcode/QRCodeReader.h>
#include <zxing/multi/qrcode/QRCodeMultiReader.h>
#include <zxing/multi/ByQuadrantReader.h>
#include <zxing/multi/MultipleBarcodeReader.h>
#include <zxing/multi/GenericMultipleBarcodeReader.h>

//#include <zxing/qrcode/detector/Detector.h>
//#include <zxing/qrcode/detector/QREdgeDetector.h>
//#include <zxing/qrcode/decoder/Decoder.h>

#include "../general_marker.h"

using namespace Magick;
using namespace std;
using namespace zxing;
using namespace zxing::multi;
using namespace zxing::qrcode;

class Marker_Zxing : public GeneralMarker {
  std::vector<Ref<Result> > decodeMultiple(Ref<BinaryBitmap> image, DecodeHints hints){
    MultiFormatReader delegate;;

    GenericMultipleBarcodeReader reader(delegate);

    return reader.decodeMultiple(image,hints);
  }

  bool tryHarder_;

public:
  Marker_Zxing():tryHarder_(false) {}

  /// retursn name of algorithm
  virtual std::string getName() const {return "marker_zxing";}

  virtual bool findPattern(const sensor_msgs::Image &img, std::vector<SMarker> &res);


  // SETTINGS
  void setTryHarder(const bool b) {tryHarder_=b;}
};

void convertPC2Magick(const pcl::PointCloud<pcl::PointXYZRGB> &in, Image &out)
{
  char buffer[128];
  sprintf(buffer,"%dx%d",(int)in.width,(int)in.height);
  out.size(buffer);
  out.magick( "RGB" );

  for(size_t x=0; x<in.width; x++)
    for(size_t y=0; y<in.height; y++)
      out.pixelColor( x, y, ColorRGB(in(x,y).r/255., in(x,y).g/255., in(x,y).b/255.) );
}

#include "impl/marker_zxing.hpp"


#endif /* PC2MAGICK_H_ */
