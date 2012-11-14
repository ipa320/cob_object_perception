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
* Project name: none
* \note
* ROS stack name: cob_object_perception
* \note
* ROS package name: cob_marker
*
* \author
* Author: Joshua Hampp
* \author
* Supervised by:
*
* \date Date of creation: 01.09.2012
*
* \brief
* cob_marker -> marker recognition with 6dof
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
