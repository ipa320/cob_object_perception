#ifndef __MAGICK_BITMAP_SOURCE_H_
#define __MAGICK_BITMAP_SOURCE_H_

#include <Magick++.h>
#include <zxing/LuminanceSource.h>

namespace zxing {

class MagickBitmapSource : public LuminanceSource {
private:
  Magick::Image image_;
  int width;
  int height;

public:
  MagickBitmapSource(Magick::Image& image);

  ~MagickBitmapSource();

  int getWidth() const;
  int getHeight() const;
  unsigned char* getRow(int y, unsigned char* row);
  unsigned char* getMatrix();
  bool isCropSupported() const;
  Ref<LuminanceSource> crop(int left, int top, int width, int height);
  bool isRotateSupported() const;
  Ref<LuminanceSource> rotateCounterClockwise();
};

}

#endif /* MAGICKMONOCHROMEBITMAPSOURCE_H_ */
