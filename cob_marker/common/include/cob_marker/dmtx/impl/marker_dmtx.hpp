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




#include <dmtx.h>


bool Marker_DMTX::findPattern(const sensor_msgs::Image &img, std::vector<SMarker> &res)
{
  int count=0;
  DmtxImage *dimg = dmtxImageCreate((unsigned char*)&img.data[0], img.width, img.height, DmtxPack24bppRGB);
  ROS_ASSERT(dimg);

  DmtxDecode *dec = dmtxDecodeCreate(dimg, 1);
  ROS_ASSERT(dec);

  dmtxDecodeSetProp(dec, DmtxPropEdgeThresh, 1);

  DmtxRegion *reg;
  
  for(count=1; count <= count_ ; count++) {
    DmtxTime timeout = dmtxTimeAdd(dmtxTimeNow(), timeout_);
    reg = dmtxRegionFindNext(dec, &timeout);
    /* Finished file or ran out of time before finding another region */
    if(reg == NULL)
    	break;

    if (reg != NULL)
    {
      DmtxMessage *msg = dmtxDecodeMatrixRegion(dec, reg, DmtxUndefined);
      if (msg != NULL)
      {
        SMarker m;
        m.code_ = std::string((const char*)msg->output,msg->outputSize);
        m.format_ = "datamatrix";

        DmtxVector2 p00, p10, p11, p01;
        p00.X = p00.Y = p10.Y = p01.X = 0.0;
        p10.X = p01.Y = p11.X = p11.Y = 1.0;
        dmtxMatrix3VMultiplyBy(&p00, reg->fit2raw);
        dmtxMatrix3VMultiplyBy(&p10, reg->fit2raw);
        dmtxMatrix3VMultiplyBy(&p11, reg->fit2raw);
        dmtxMatrix3VMultiplyBy(&p01, reg->fit2raw);

        Eigen::Vector2i v;
        v(0)=(int)((p01.X) + 0.5);
        v(1)=img.height - 1 - (int)((p01.Y) + 0.5);
        m.pts_.push_back(v);

        v(0)=(int)((p00.X) + 0.5);
        v(1)=img.height - 1 - (int)((p00.Y) + 0.5);
        m.pts_.push_back(v);

        v(0)=(int)((p11.X) + 0.5);
        v(1)=img.height - 1 - (int)((p11.Y) + 0.5);
        m.pts_.push_back(v);

        v(0)=(int)((p10.X) + 0.5);
        v(1)=img.height - 1 - (int)((p10.Y) + 0.5);
        m.pts_.push_back(v);

        res.push_back(m);

        dmtxMessageDestroy(&msg);
      }
      dmtxRegionDestroy(&reg);
    }
  }

  dmtxDecodeDestroy(&dec);
  dmtxImageDestroy(&dimg);

  return false;
}
