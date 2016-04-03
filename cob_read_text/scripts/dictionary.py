import string
import sys

def dictionary(filename,output):
	''' this python script read a plain text dictionary, generate lower, upper and captial cases words into another file'''
	f = open(filename, 'r')
	g = open(output,'w')
	for line in f:
		cap = string.capitalize(line)
		low = string.lower(line)
		up = string.upper(line)
		g.write(low + up + cap)
	
if __name__ == "__main__":
	dictionary(sys.argv[1] , sys.argv[2])
