/*
 * marker_dmtx.h
 *
 *  Created on: 23.08.2012
 *      Author: josh
 */

#ifndef MARKER_DMTX_H_
#define MARKER_DMTX_H_

#include "../general_marker.h"

class Marker_DMTX : public GeneralMarker {

  long timeout_;

public:
  Marker_DMTX():timeout_(100) {}

  /// retursn name of algorithm
  virtual std::string getName() const {return "marker_dmtx";}

  virtual bool findPattern(const sensor_msgs::Image &img, std::vector<SMarker> &res);


  // SETTINGS
  void setTimeout(const long t) {timeout_=t;}
};

#include "impl/marker_dmtx.hpp"


#endif /* MARKER_DMTX_H_ */
