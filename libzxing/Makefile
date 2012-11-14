include $(shell rospack find mk)/cmake.mk

all:
	unlink include || echo "start"
	unlink lib/* || echo "start"
	unzip -o build/ZXing.zip -d zxing && cd zxing/zxing*/core && ant &&  cd ../javase && ant && cd ../cpp && scons lib && ln -s `pwd`/core/src ../../../include && mkdir -p ../../../lib && ln -s `pwd`/build/*.a ../../../lib/
