PYVERS = "`python -c 'import sys; print sys.version[:3]'`"

all:
	python setup.py build

install:
	python setup.py install --prefix=build
	mkdir -p ../src
	cp build/lib/python$(PYVERS)/site-packages/* ../src

clean:
	rm -Rf build
	rm -f *.pyc hello.png
	rm -Rf ../src

.PHONY: all install clean
