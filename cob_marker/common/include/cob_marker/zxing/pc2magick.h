/*
 * pc2magick.h
 *
 *  Created on: 16.08.2012
 *      Author: josh
 */

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
