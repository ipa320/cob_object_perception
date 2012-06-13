all:
	python setup.py build

install:
	python setup.py install --prefix=build
	mkdir -p ../src
	cp build/lib/python2.6/site-packages/* ../src

clean:
	rm -Rf build
	rm -f *.pyc hello.png

.PHONY: all install clean
