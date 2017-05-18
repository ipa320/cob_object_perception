import os
import os.path
import getopt 
import sys

def filelist(thepath):
	'''		this python script read images from a directory, 
				add perform text detection	'''
	dirlist = os.listdir(thepath)
	executeBin = "$(rospack find cob_read_text)/bin/run_detect "
	correlation = "$(rospack find cob_read_text)/fonts/correlation.txt"
	dictionary = "$(rospack find cob_read_text)/dictionary/full-dictionary"
	print "current directory:" + thepath
	for name in dirlist:
		absolutePath = os.path.join(thepath,name)
		if not  os.path.isdir(absolutePath):
			print name
			if name.endswith(".jpg") or name.endswith(".png") or name.endswith(".JPG"):
				execute = executeBin + absolutePath + correlation + dictionary
				os.system(execute)
		else:
			filelist(absolutePath)

if __name__ == "__main__":
	filelist(sys.argv[1]);
