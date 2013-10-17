/*
 * SerializeIO.h
 *
 *  Created on: 20.06.2013
 *      Author: Matthias Nösner
 */

#ifndef SERIALIZEIO_H_
#define SERIALIZEIO_H_
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <cstring>

struct keyword{
	unsigned int size;
	std::string  keyw_str;
	std::string  ekeyw_str;

	keyword(char const* w){
		std::stringstream ss;
		//opening Keys
		ss << w;
		keyw_str = ss.str();
		size = keyw_str.size();

		ss.clear();
		ss.str(std::string());

		//ending keys
		ss << '/' << w;
		ekeyw_str = ss.str();
	}
};


class SerializeIO {
public:
	SerializeIO();
	SerializeIO(char const*,char);
	virtual ~SerializeIO();

	void writeVariable(char const*, int);
	void writeVariable(char const*, float);
	void writeVariable(char const*, double);

	bool openArray(char const* name);
	bool closeArray();

	void writeVector(std::vector<int> vec);
	void writeVector(std::vector<float> vec);
	void writeVector(std::vector<double> vec);

	bool readVariable(char const*,unsigned int*);
	bool readVariable(char const*,int*);
	bool readVariable(char const*,float*);
	bool readVariable(char const*,double*);

	bool readArray(char const*, std::vector<std::vector<int> >*);
	bool readArray(char const*, std::vector<std::vector<float> >*);
	bool readArray(char const*, std::vector<std::vector<double> >*);

	//------Experimental------
	bool readArrayEXP(char const*, std::vector<std::vector<int> >*);
	bool readArrayEXP(char const*, std::vector<std::vector<double> >*);
	bool writeVectorEXP(std::vector<int> vec);
	bool writeVectorEXP(std::vector<double> vec);
	void writeVectorToFileEXP(std::vector<double> vec);
	//------Experimental------

	void close();

private:
	std::fstream file;
	std::vector<keyword> keywords;
	bool arrayopen;
	unsigned int arrayindex;
	unsigned int max_chars;

	void writeVariableToFile(std::string name,std::string value);
	const char* prepareChar(char*);

	bool parse(char**,char const*);
	bool chooseInfoParser(char**,char const*,char const*,keyword);
	bool parseVar(char**,char const*,char const*);
	bool parseKey(char**,char const*,char const*,keyword);

	void chopToClosing(char const*,char const*,keyword);
	unsigned int findOpening(const char*,keyword,unsigned int);
	unsigned int findClosing(const char*,keyword,unsigned int);
	bool compareName(char const*,char const*,unsigned int);

	bool readVarFromCchar(char const*,char const*,int*);
	bool readVarFromCchar(char const*,char const*,float*);
	bool readVarFromCchar(char const*,char const*,double*);

	void writeVectorToFile(std::vector<double> vec);


};

#endif /* SERIALIZEIO_H_ */
