include $(shell rospack find mk)/cmake.mk

all:
	mkdir -p build && wget -O build/ZXing.zip http://zxing.googlecode.com/files/ZXing-2.1.zip
	unlink include || echo "start"
	unlink lib/* || echo "start"
	unzip -o build/ZXing.zip -d zxing && cd zxing/zxing*/core && ant &&  cd ../javase && ant && cd ../cpp && scons lib && ln -s `pwd`/core/src ../../../include && mkdir -p ../../../lib && ln -s `pwd`/build/*.a ../../../lib/
