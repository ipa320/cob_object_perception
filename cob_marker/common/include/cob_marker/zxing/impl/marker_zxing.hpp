/*
 * marker_zxing.hpp
 *
 *  Created on: 22.08.2012
 *      Author: josh
 */



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
