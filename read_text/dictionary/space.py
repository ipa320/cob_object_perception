#!/usr/bin/python
fobj = open("full-dictionary", "r")
fobj2 = open("new-dictionary", "w")
for line in fobj: 
	fobj2.write(" " + line) 
fobj.close()
fobj2.close()
