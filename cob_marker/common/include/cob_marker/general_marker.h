/*
 * general_marker.h
 *
 *  Created on: 21.08.2012
 *      Author: josh
 */

#ifndef GENERAL_MARKER_H_
#define GENERAL_MARKER_H_


class GeneralMarker {
public:
  struct SMarker {
    std::string code_;  //marker content like "stone"
    std::string format_;  //marker format like "Qr"
    std::vector<Eigen::Vector2i> pts_; //points in color image
  };

  virtual ~GeneralMarker() {}

  /// retursn name of algorithm
  virtual std::string getName() const = 0;

  virtual bool findPattern(const sensor_msgs::Image &img, std::vector<SMarker> &res)=0;
};

#include "zxing/pc2magick.h"
#include "dmtx/marker_dmtx.h"


#endif /* GENERAL_MARKER_H_ */
