/*
 * marker_dmtx.hpp
 *
 *  Created on: 23.08.2012
 *      Author: josh
 */


#include <dmtx.h>


bool Marker_DMTX::findPattern(const sensor_msgs::Image &img, std::vector<SMarker> &res)
{
  DmtxImage *dimg = dmtxImageCreate((unsigned char*)&img.data[0], img.width, img.height, DmtxPack24bppRGB);
  ROS_ASSERT(dimg);

  DmtxDecode *dec = dmtxDecodeCreate(dimg, 1);
  ROS_ASSERT(dec);

  dmtxDecodeSetProp(dec, DmtxPropEdgeThresh, 1);

  DmtxRegion *reg;

  do {
    DmtxTime timeout = dmtxTimeAdd(dmtxTimeNow(), timeout_);
    reg = dmtxRegionFindNext(dec, &timeout);
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

        std::cout<<m.code_<<"\n";
        res.push_back(m);

        dmtxMessageDestroy(&msg);
      }
      dmtxRegionDestroy(&reg);
    }
  } while(reg);

  dmtxDecodeDestroy(&dec);
  dmtxImageDestroy(&dimg);

  return false;
}
