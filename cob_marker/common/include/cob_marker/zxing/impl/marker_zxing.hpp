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




bool Marker_Zxing::findPattern(const sensor_msgs::Image &img, std::vector<SMarker> &res)
{
  //convert
  Image mgck;

  char buffer[128];
  sprintf(buffer,"%dx%d",(int)img.width,(int)img.height);
  mgck.size(buffer);
  mgck.magick( "RGB" );

  int step = img.step/img.width;
  ROS_ASSERT(step==3);

  for(size_t x=0; x<img.width; x++)
    for(size_t y=0; y<img.height; y++)
      mgck.pixelColor( x, y, ColorRGB(
          img.data[(y*img.width+x)*step]/255.,
          img.data[(y*img.width+x)*step+1]/255.,
          img.data[(y*img.width+x)*step+2]/255.) );

  //search
  vector<Ref<Result> > results;
  string cell_result;
  int r = -1;

  Ref<BitMatrix> matrix(NULL);
  Ref<Binarizer> binarizer(NULL);

  try {
    Ref<MagickBitmapSource> source(new MagickBitmapSource(mgck));

    binarizer = new HybridBinarizer(source);

    DecodeHints hints(DecodeHints::DEFAULT_HINT);
    hints.setTryHarder(tryHarder_);
    Ref<BinaryBitmap> binary(new BinaryBitmap(binarizer));
    results = decodeMultiple(binary, hints);
    r = 0;
  } catch (ReaderException e) {
    cell_result = "zxing::ReaderException: " + string(e.what());
    r = -2;
  } catch (zxing::IllegalArgumentException& e) {
    cell_result = "zxing::IllegalArgumentException: " + string(e.what());
    r = -3;
  } catch (zxing::Exception& e) {
    cell_result = "zxing::Exception: " + string(e.what());
    r = -4;
  } catch (std::exception& e) {
    cell_result = "std::exception: " + string(e.what());
    r = -5;
  }

  if (r != 0){
    cout<<" binarizer failed: "<<cell_result<<endl;
  } else {
    for (unsigned int i = 0; i < results.size(); i++){
      cout << "    "<<results[i]->getText()->getText();
      cout << " " << barcodeFormatNames[results[i]->getBarcodeFormat()];
      cout << endl;

      SMarker m;
      m.code_ = results[i]->getText()->getText();
      m.format_ = barcodeFormatNames[results[i]->getBarcodeFormat()];

      for(size_t j=0; j<results[i]->getResultPoints().size(); j++) {
        Eigen::Vector2i p;
        p(0) = results[i]->getResultPoints()[j]->getX();
        p(1) = results[i]->getResultPoints()[j]->getY();
        m.pts_.push_back(p);
      }

      res.push_back(m);
    }

    return true;
  }

  return false;
}
