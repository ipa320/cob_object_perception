/*
 * SerializeIO.cpp
 *
 *  Created on: 20.06.2013
 *      Author: Matthias Nösner
 */
//TODO: Ability to write some Information (Documentation) to the File. Especially a header for Arrays to name the Data in each colum


#include "SerializeIO.h"
using namespace std;

//---------------------Constructor/Destructor---------------------------------------------------------------------------------------------------------
// 'i' -> infile(read from file) , 'o' -> outfile(write to file)
SerializeIO::SerializeIO(char const* filename, char io) {

	if(io == 'i') {
		file.open(filename,ios_base::in);
	}
	else if(io =='o'){
		file.open(filename,ios_base::out);
	}
	else {
		cout << "SerializeIO: Failure in SerializeIO(char const* filename, char io) -> io should be 'i' or 'o'" << endl;
	}

	if(!file.is_open()){
		cout << "SerializeIO: SerializeIO(char const* filename, char io): Could not open file.. check the filename -> Shutdown Program" << endl;
		exit(EXIT_FAILURE);
	}

	//DONT TOUCH ORDERING!!!!!!!!
	keywords.push_back(keyword("var"));
	keywords.push_back(keyword("array"));
	keywords.push_back(keyword("set"));

	arrayopen = false;
	arrayindex = 0;

	max_chars = 2048;
}

SerializeIO::~SerializeIO() {

}
//--------------------------------------------openArray() && closeArray()------------------------------------------------------------------------------------------
//Need to call before and after writing to an Array
bool SerializeIO::openArray(char const* name){

	stringstream ss;
	if(!arrayopen){
		ss << "<array " << "name=\"" << name << "\">" << "\n";
		file << ss.str();
		arrayindex = 0;
		arrayopen = true;
		return true;
	} else {
		cout << "SerializeIO::openArray(string name): " << "Could not open Array. There's already one open" << endl;
	}

	return false;
}

bool SerializeIO::closeArray(){

	stringstream ss;
	if(arrayopen){
		ss << "</array>" << "\n";
		file << ss.str();
		arrayopen = false;
		return true;
	} else {
		cout << "SerializeIO::closeArray(): " << "Could not close Array. No Array is open" << endl;
	}

	return false;
}


//--------------------------------------------writeVariableToFile()------------------------------------------------------------------------------------------
void SerializeIO::writeVariableToFile(string name,string value){

	stringstream ss;
	ss << "<var " << "name=\"" << name << "\">" << value << "</var>" << "\n";
	file << ss.str();
}



//--------------------------------------------writeVariable()------------------------------------------------------------------------------------------

void SerializeIO::writeVariable(char const* name, int value){

	stringstream ss;

	ss << name;
	string sname = ss.str();
	ss.clear();
	ss.str(string());
	ss << value;
	string svalue = ss.str();

	writeVariableToFile(sname,svalue);
}
void SerializeIO::writeVariable(char const* name, float value){

	stringstream ss;

	ss << name;
	string sname = ss.str();
	ss.clear();
	ss.str(string());
	ss << value;
	string svalue = ss.str();

	writeVariableToFile(sname,svalue);
}
void SerializeIO::writeVariable(char const* name, double value){

	stringstream ss;

	ss << name;
	string sname = ss.str();
	ss.clear();
	ss.str(string());
	ss << value;
	string svalue = ss.str();

	writeVariableToFile(sname,svalue);
}
//--------------------------------------------readVariable()------------------------------------------------------------------------------------------
//public Methods for reading a var with the given name tofind
bool SerializeIO::readVariable(char const* tofind, unsigned int* val){

	char *parsed_data = NULL;
	bool ret = parse(&parsed_data,tofind);

	if(ret)
		*val = (unsigned int)atof(prepareChar(parsed_data));
	else
		cout << "readVariable(char const* tofind, unsigned int* val): " << "No such Variable" << endl;

	delete parsed_data;
	return ret;
}

bool SerializeIO::readVariable(char const* tofind, int* val){

	char *parsed_data = NULL;
	bool ret = parse(&parsed_data,tofind);

	if(ret)
		*val = (int)atof(prepareChar(parsed_data));
	else
		cout << "readVariable(char const* tofind, int* val): " << "No such Variable" << endl;

	return ret;
}
bool SerializeIO::readVariable(char const* tofind, float* val){

	char *parsed_data = NULL;
	bool ret = parse(&parsed_data,tofind);

	if(ret)
		*val = (float)atof(prepareChar(parsed_data));
	else
		cout << "readVariable(char const* tofind, float* val): " << "No such Variable" << endl;

	return ret;
}
bool SerializeIO::readVariable(char const* tofind, double* val){

	char *parsed_data = NULL;
	bool ret = parse(&parsed_data,tofind);

	if(ret)
		*val = (double)atof(prepareChar(parsed_data));
	else
		cout << "readVariable(char const* tofind, double* val): " << "No such Variable" << endl;

	return ret;
}

//--------------------------------------------readVarFromCchar()------------------------------------------------------------------------------------------
//private methods to read variables from char const*
bool SerializeIO::readVarFromCchar(char const* toparse,char const* tofind, int* val){

	char *parsed_data = NULL;

	bool ret = parseVar(&parsed_data,toparse,tofind);

	if(ret)
		*val = (int)atof(prepareChar(parsed_data));

	return ret;
}

bool SerializeIO::readVarFromCchar(char const* toparse,char const* tofind, float* val){

	char *parsed_data = NULL;

	bool ret = parseVar(&parsed_data,toparse,tofind);

	if(ret)
		*val = atof(prepareChar(parsed_data));

	return ret;
}


bool SerializeIO::readVarFromCchar(char const* toparse,char const* tofind, double* val){

	char *parsed_data = NULL;

	bool ret = parseVar(&parsed_data,toparse,tofind);

	if(ret)
		*val = (double)atof(prepareChar(parsed_data));

	return ret;
}

//--------------------------------------------READ ARRAY-----------------------------------------------------------------------------------
//WORK
//TODO '\n' is missing in parsed_data.... -> workaround with line delimiter |
bool SerializeIO::readArray(char const* tofind, std::vector<std::vector<int> >* vec){

	vector<int> v;
	char *ctmp = new char[255];

	char *parsed_data = NULL;
	parse(&parsed_data,tofind);

	//Run through parse_data -> All stuff between <array></array>
	unsigned int scount = 0;//act. pose in Variable to read
	for(unsigned int i = 0; parsed_data[i] != '\0';i++){

		if(parsed_data[i] != ' ' && parsed_data[i] != '|'){
			ctmp[scount] = parsed_data[i];
			scount++;
		}

		if(parsed_data[i] == ' ' || parsed_data[i] == '|'){
			ctmp[scount] = '\0';
			v.push_back(atoi(prepareChar(ctmp)));
			scount = 0;
		}

		//Reset for new Line
		if(parsed_data[i] == '|'){
			scount = 0;
			vec->push_back(v);
			v.clear();
		}
	}
	return true;
}

bool SerializeIO::readArray(char const* tofind, std::vector<std::vector<float> >* vec){

	vector<float> v;
	char *ctmp = new char[255];

	char *parsed_data = NULL;
	parse(&parsed_data,tofind);

	//Run through parse_data -> All stuff between <array></array>
	unsigned int scount = 0;//act. pose in Variable to read
	for(unsigned int i = 0; parsed_data[i] != '\0';i++){

		if(parsed_data[i] != ' ' && parsed_data[i] != '|'){
			ctmp[scount] = parsed_data[i];
			scount++;
		}

		if(parsed_data[i] == ' ' || parsed_data[i] == '|'){
			ctmp[scount] = '\0';
			v.push_back(atof(prepareChar(ctmp)));
			scount = 0;
		}

		//Reset for new Line
		if(parsed_data[i] == '|'){
			scount = 0;
			vec->push_back(v);
			v.clear();
		}
	}
	return true;
}

bool SerializeIO::readArray(char const* tofind, std::vector<std::vector<double> >* vec){

	vector<double> v;
	char *ctmp = new char[255];

	char *parsed_data = NULL;
	parse(&parsed_data,tofind);

	//Run through parse_data -> All stuff between <array></array>
	unsigned int scount = 0;//act. pose in Variable to read
	for(unsigned int i = 0; parsed_data[i] != '\0';i++){

		if(parsed_data[i] != ' ' && parsed_data[i] != '|'){
			ctmp[scount] = parsed_data[i];
			scount++;
		}

		if(parsed_data[i] == ' ' || parsed_data[i] == '|'){
			ctmp[scount] = '\0';
			v.push_back((double)atof(prepareChar(ctmp)));
			scount = 0;
		}

		//Reset for new Line
		if(parsed_data[i] == '|'){
			scount = 0;
			vec->push_back(v);
			v.clear();
		}
	}
	return true;
}

//------------------------------------------write vector--------------------------------------------------------------------------------------------
//Write a vector with different data types to an array
//Each vector is one line in the array
bool SerializeIO::writeVector(std::vector<int> vec){

	std::vector<double> vecd;

	for(unsigned int i=0; i<vec.size(); i++){
		vecd.push_back((double)vec[i]);
	}
	writeVectorToFile(vecd);
	return true;
}

bool SerializeIO::writeVector(std::vector<float> vec){

	std::vector<double> vecd;

	for(unsigned int i=0; i<vec.size(); i++){
		vecd.push_back((double)vec[i]);
	}
	writeVectorToFile(vecd);
	return true;
}

bool SerializeIO::writeVector(std::vector<double> vec){

	std::vector<double> vecd;

	for(unsigned int i=0; i<vec.size(); i++){
		vecd.push_back(vec[i]);
	}
	writeVectorToFile(vecd);
	return true;
}

//------------------------------------------write vector to FIle--------------------------------------------------------------------------------------------
//parses the data to a stringstream and writes the data to file
void SerializeIO::writeVectorToFile(std::vector<double> vec){

	stringstream ss;
	ss << arrayindex;
	string index = ss.str();
	ss.str(string());

	for(unsigned int i = 0; i < vec.size(); i++){
		ss << vec[i];
		if(i < vec.size()-1)
			ss << ' ';
	}

	ss << '|' << '\n';

	file << ss.str();
	arrayindex++;
}


//--------------------------------------------prepareChar()---------------------------------------------------------------------------------------------
// prepare a char before convertion to native type like int,float
// filter wrong characters
const char* SerializeIO::prepareChar(char* toparse){

	//make these evt.golobal.. cause escape seuqences will be used from multiple functions
	const char escapesequence[] = "0123456789-.,";

	stringstream ss;

	bool warning = false;
	for(unsigned int i = 0; toparse[i] != '\0';i++){
		bool tmpwarning = true;
		for(unsigned int j = 0; escapesequence[j] != '\0';j++){
			if(toparse[i] == escapesequence[j]){
				ss << toparse[i];
				tmpwarning = false;
			}
		}
		if(tmpwarning) warning = true;
	}

	if(warning){
		cout << "WARNING: SerializeIO::prepareChar(char* toparse): Wrong sings in char* toparse. I escaped the wrong signs, but YOU are responsible for errors!!" << endl;
	}

	return ss.str().c_str();
}


//--------------------------------------------parse()------------------------------------------------------------------------------------------
// reads the complete file and searchs for keywords and stores the Data of this element in parsed_data
// for further computation it calls chooseInfoParse()
// Hint: Parse will not search in dimensions of other keywords eg. <array><var></var></array> -> var will not be noticed!!
bool SerializeIO::parse(char **parsed_data, char const* tofind){
	file.clear() ;
	file.seekg(0, ios::beg) ;

	vector<char*>  datafile;
	datafile.push_back(new char[max_chars]);

	//Read filedata into vector<char *>
	for(int i=0; file.getline(datafile[i],max_chars); i++){
		datafile.push_back(new char[max_chars]);
	}
	datafile.pop_back();  //Pop last element.. cause there's no data in it

	//step through lines
	for(unsigned int i = 0; i < datafile.size(); i++){
		//step throug letters
		for( unsigned int j = 0; datafile[i][j] != '\0'; j++){
			if(datafile[i][j] == '<'){
				//compare with keywords
				for(unsigned int k=0; k < keywords.size(); k++){
					stringstream ss;
					for(unsigned int h=1; h <= keywords[k].size; h++){
						ss << datafile[i][j+h];
					}
					//compare
					if(!keywords[k].keyw_str.compare(ss.str())){

						//Iteration to detect ending sign
						bool outerloop = true;
						for(unsigned int m=i; m < datafile.size() && outerloop; m++){
							unsigned int n = 0;//start from begin of row
							if(m==i) n=j+1; //start from last processed sign

							for(; datafile[m][n] != '\0'; n++){
								if(datafile[m][n] == '<'){
									//compare with endkey
									stringstream ss_;
									for(unsigned int p=1; p <= keywords[k].ekeyw_str.size(); p++){
										ss_ << datafile[m][n+p];
									}

									if(!keywords[k].ekeyw_str.compare(ss_.str())){

										stringstream ss__;
										//read area with opening and closing signs <xxx and </xxx> ->xxx = keywords
										for(;i <= m;i++){
											if(i<m){
												for(;datafile[i][j] != '\0';j++){
													ss__ << datafile[i][j];
												}
											}
											j=0;

											if(i==m){
												// include the closing sign!! -> will be escaped by parseVar
												for(;j<n+keywords[k].ekeyw_str.size()+2;j++){
													ss__ << datafile[i][j];
												}
											}
										}
										i--; //adjust main loop
										ss__ << '\0';  //delimiter... -> Without.. no chance to find end of char

										if(chooseInfoParser(parsed_data,tofind,ss__.str().c_str(),keywords[k])){
											return true;
										}

										outerloop = false;
										break;
									}
								}
							}
						}
					}
				}
			}
		}
	}

	//free memory
	for(unsigned int i = 0; i < datafile.size(); i++){
		delete datafile[i];
	}

	return false;
}

//--------------------------------------------------chooseInfoParser()-------------------------------------------------------------------------------
//choses the further parsing strategy in dependence of the keyword
bool SerializeIO::chooseInfoParser(char **parsed_data,char const* tofind,char const* toparse,keyword key){

		return parseVar(parsed_data,toparse,tofind);
}

//--------------------------------------------------parseVar()---------------------------------------------------------------------------------------
//parse value between <var name"xxx">value</val>
bool  SerializeIO::parseVar(char **parsed_data, char const* toparse, char const* tofind){

	stringstream ss;
	string kname = "name";

	//cout << toparse << endl;

	//search kname
	bool extractvalue = false;
	unsigned int i = 0;
	for(; toparse[i] != '>';i++){
		stringstream ss_;
		unsigned int j = 1;
		for(; j <= kname.size(); j++){
			ss_ << toparse[i+j];
		}
		if(!kname.compare(ss_.str())){
			//jump to end of name
			i = i + j;
			extractvalue = true;
			break;
		}
	}

	//extract value of name="value" between "" or ''
	if(extractvalue){
		bool reading = false;

		for(;toparse[i] != '>';i++){
			bool limiter = (toparse[i] == '\"' || toparse[i] == '\'');
			if(limiter){
				if(!reading)
					reading = true;
				else
					reading = false;
			}

			if(reading && !limiter)
				ss << toparse[i];
		}
	}

	//delete closing sign </var> or </array> or whatever
	// search from end of const char*
	unsigned int chrlength = 0;
	for(;toparse[chrlength] != '\0';chrlength++){}
	for(;toparse[chrlength] != '<';chrlength--){}

	//compare & read value
	if(!ss.str().compare(tofind)){
		//read Value... would be also used on other places
		//TODO: maybe Filter out empty space
		stringstream ss__;
		for(unsigned int m=i+1; m < chrlength ;m++)
			ss__ << toparse[m];

		ss__ << '\0';

		//allocate memory and convert const char* to char*
		*parsed_data = new char[ss__.str().length()];
		strcpy(*parsed_data,ss__.str().c_str());
		return true;
	}

	*parsed_data = new char('\0');
	return false;
}

//--------------------------------------------------parseKey()------------------------------------------------------------------------------------------
//cuts char array to searched signs
bool SerializeIO::parseKey(char** parsed_data,char const* toparse,char const* tofind,keyword keyw){

	//cout << tofind << endl;

	for(unsigned int i = 0; toparse[i] != '\0';i++){
		if(toparse[i] == '<'){
			unsigned int open = findOpening(toparse,keyw,i);
			unsigned int close = findClosing(toparse,keyw,open);

			if(compareName(toparse,tofind,open)){
				stringstream ss;
				for(unsigned j = open ; j <= close;j++)
					ss << toparse[j];

				ss << '\0';
				*parsed_data = new char[ss.str().length()];
				cout << "1111   " << ss.str().length() << ":::::" << ss.str() << endl;
				strcpy(*parsed_data,ss.str().c_str());
				return true;
			}

			i = close;
		}
	}

	*parsed_data = new char('\0');
	return false;
}
//--------------------------------------------------Helper------------------------------------------------------------------------------------------

unsigned int SerializeIO::findOpening(const char* toparse, keyword keyw, unsigned int cnt){
	for(; toparse[cnt] != '\0';cnt++){
		if(toparse[cnt] == '<'){
			stringstream ss_open;

			for(unsigned int p=1; p <= keyw.keyw_str.size(); p++){
				ss_open << toparse[cnt+p];
			}
			//opening?
			if(!keyw.keyw_str.compare(ss_open.str())){
				return cnt;
			}
		}
	}
	return cnt;
}

unsigned int SerializeIO::findClosing(const char* toparse, keyword keyw, unsigned int cnt){

	for(; toparse[cnt] != '\0';cnt++){
		if(toparse[cnt] == '<'){
			stringstream ss_close;

			for(unsigned int p=1; p <= keyw.ekeyw_str.size(); p++){
				ss_close << toparse[cnt+p];
			}

			//closing?
			if(!keyw.ekeyw_str.compare(ss_close.str())){
				return cnt+keyw.ekeyw_str.size()+1;
			}
		}
	}
	return cnt;
}

//need opening pos (cnt) of an element
bool SerializeIO::compareName(char const* toparse,char const* tofind,unsigned int cnt){

	stringstream ss;
	string kname = "name";

	//search kname
	bool extractvalue = false;
	unsigned int i = cnt;
	for(; toparse[i] != '>';i++){
		stringstream ss_;
		unsigned int j = 1;
		for(; j <= kname.size(); j++){
			ss_ << toparse[i+j];
		}
		if(!kname.compare(ss_.str())){
			//jump to end of name
			i = i + j;
			extractvalue = true;
			break;
		}
	}

	//extract value of name="value" between "" or ''
	if(extractvalue){
		bool reading = false;

		for(;toparse[i] != '>';i++){
			bool limiter = (toparse[i] == '\"' || toparse[i] == '\'');
			if(limiter){
				if(!reading)
					reading = true;
				else
					reading = false;
			}

			if(reading && !limiter)
				ss << toparse[i];
		}
	}
	if(!ss.str().compare(tofind)){
		return true;
	}

	return false;
}

//--------------------------------------------------close()------------------------------------------------------------------------------------------
void SerializeIO::close(){
	file.close();
}



//---------------------------------------------DEPRECATED STUFF--------------------------------------------------------------------------------------------

//--------------------------------------------writeVectorToFile()------------------------------------------------------------------------------------------
//!!!!!!!Deprecated!!!!!!!!!
void SerializeIO::writeVectorToFileDEP(std::vector<double> vec){

	stringstream ss;
	ss << arrayindex;
	string index = ss.str();
	ss.str(string());

	ss << "<set " << "name=\"" << index << "\">";
	for(unsigned int i = 0; i < vec.size(); i++)
		ss << "<var " << "name=\"" << i << "\">" << vec[i] << "</var>";
	ss << "</set>" << "\n";

	file << ss.str();
	arrayindex++;
}

//--------------------------------------------writeVecvtor()------------------------------------------------------------------------------------------
//!!!!!!!Deprecated!!!!!!!!!
//Write the lines of an Array
void SerializeIO::writeVectorDEP(std::vector<int> vec){

	std::vector<double> vecd;

	for(unsigned int i=0; i<vec.size(); i++){
		vecd.push_back((double)vec[i]);
	}
	writeVectorToFileDEP(vecd);
}
void SerializeIO::writeVectorDEP(std::vector<float> vec){

	std::vector<double> vecd;

	for(unsigned int i=0; i<vec.size(); i++){
		vecd.push_back((double)vec[i]);
	}
	writeVectorToFileDEP(vecd);
}
void SerializeIO::writeVectorDEP(std::vector<double> vec){

	writeVectorToFileDEP(vec);
}

//--------------------------------------------readArray()------------------------------------------------------------------------------------------
//!!!!!!!Deprecated!!!!!!!!!
//Overloaded function to read array data structure

//Overloaded with int
bool SerializeIO::readArrayDEP(char const* tofind, std::vector<std::vector<int> >* vec){

	vector<int> v;

	char *parsed_data = NULL;
	parse(&parsed_data,tofind);

	//**************************************************************************************** <set><var>xxx</var></set>
	stringstream ss;
	for(unsigned int i = 0; parsed_data[i] != '\0';i++)
		ss << parsed_data[i];

	char *parsed_data_set = NULL;

	int row = 0;
	stringstream ss_;
	ss_ << row;
	// Run through <set></set>
	while(parseKey(&parsed_data_set,ss.str().c_str(),ss_.str().c_str(),keywords[2])){ // Be patient with keywords

		//*********************************************************** <var>xxx</var>
		stringstream ss___;
		for(unsigned int i = 0; parsed_data_set[i] != '\0';i++)
			ss___ << parsed_data_set[i];


		std::vector<int> var;
		char *parsed_data_var = NULL;
		int elem = 0;
		stringstream ss__;
		ss__ << elem;
		//Run through <var name="0"></var> <var name="1"></var> etc.
		while(parseKey(&parsed_data_var,ss___.str().c_str(),ss__.str().c_str(),keywords[0])){ // Be patient with keywords

			//******************************************* xxx
				stringstream ss____;
				for(unsigned int i = 0; parsed_data_var[i] != '\0';i++)
					ss____ << parsed_data_var[i];

				int val = 0;
				if(!readVarFromCchar(ss____.str().c_str(),ss__.str().c_str(),&val)) return false;

				var.push_back(val);//save var to row vec

			//******************************************* xxx

			//prepare for next iteration
			elem++;
			ss__.str(string());
			ss__ << elem;
			parsed_data_set = NULL;
		}

		vec->push_back(var); // save row vec to list

		//*********************************************************** <var>xxx</var>
		//prepare for next iteration
		row++;
		ss_.str(string());
		ss_ << row;
		parsed_data_set = NULL;
	}
	//************************************************************************************************ <set><var>xxx</var></set>
	return true;
}

//Overloaded with float
bool SerializeIO::readArrayDEP(char const* tofind, std::vector<std::vector<float> >* vec){

	vector<int> v;

	char *parsed_data = NULL;
	parse(&parsed_data,tofind);

	//**************************************************************************************** <set><var>xxx</var></set>
	stringstream ss;
	for(unsigned int i = 0; parsed_data[i] != '\0';i++)
		ss << parsed_data[i];

	char *parsed_data_set = NULL;

	int row = 0;
	stringstream ss_;
	ss_ << row;
	// Run through <set></set>
	while(parseKey(&parsed_data_set,ss.str().c_str(),ss_.str().c_str(),keywords[2])){ // Be patient with keywords

		//*********************************************************** <var>xxx</var>
		stringstream ss___;
		for(unsigned int i = 0; parsed_data_set[i] != '\0';i++)
			ss___ << parsed_data_set[i];


		std::vector<float> var;
		char *parsed_data_var = NULL;
		int elem = 0;
		stringstream ss__;
		ss__ << elem;
		//Run through <var name="0"></var> <var name="1"></var> etc.
		while(parseKey(&parsed_data_var,ss___.str().c_str(),ss__.str().c_str(),keywords[0])){ // Be patient with keywords

			//******************************************* xxx
				stringstream ss____;
				for(unsigned int i = 0; parsed_data_var[i] != '\0';i++)
					ss____ << parsed_data_var[i];

				float val = 0;
				if(!readVarFromCchar(ss____.str().c_str(),ss__.str().c_str(),&val)) return false;

				var.push_back(val);//save var to row vec

			//******************************************* xxx

			//prepare for next iteration
			elem++;
			ss__.str(string());
			ss__ << elem;
			parsed_data_set = NULL;
		}

		vec->push_back(var); // save row vec to list

		//*********************************************************** <var>xxx</var>
		//prepare for next iteration
		row++;
		ss_.str(string());
		ss_ << row;
		parsed_data_set = NULL;
	}
	//************************************************************************************************ <set><var>xxx</var></set>
	return true;
}

//Overloaded with double
bool SerializeIO::readArrayDEP(char const* tofind, std::vector<std::vector<double> >* vec){

	vector<int> v;

	char *parsed_data = NULL;
	parse(&parsed_data,tofind);

	//**************************************************************************************** <set><var>xxx</var></set>
	stringstream ss;
	for(unsigned int i = 0; parsed_data[i] != '\0';i++)
		ss << parsed_data[i];

	char *parsed_data_set = NULL;
	int row = 0;
	stringstream ss_;
	ss_ << row;
	// Run through <set></set>
	while(parseKey(&parsed_data_set,ss.str().c_str(),ss_.str().c_str(),keywords[2])){ // Be patient with keywords

		//*********************************************************** <var>xxx</var>
		stringstream ss___;
		for(unsigned int i = 0; parsed_data_set[i] != '\0';i++)
			ss___ << parsed_data_set[i];


		std::vector<double> var;
		char *parsed_data_var = NULL;
		int elem = 0;
		stringstream ss__;
		ss__ << elem;
		//Run through <var name="0"></var> <var name="1"></var> etc.
		while(parseKey(&parsed_data_var,ss___.str().c_str(),ss__.str().c_str(),keywords[0])){ // Be patient with keywords

			//******************************************* xxx
				stringstream ss____;
				for(unsigned int i = 0; parsed_data_var[i] != '\0';i++)
					ss____ << parsed_data_var[i];

				double val = 0;
				if(!readVarFromCchar(ss____.str().c_str(),ss__.str().c_str(),&val)) return false;

				var.push_back(val);//save var to row vec

			//******************************************* xxx

			//prepare for next iteration
			elem++;
			ss__.str(string());
			ss__ << elem;
			parsed_data_set = NULL;
		}

		vec->push_back(var); // save row vec to list
		//*********************************************************** <var>xxx</var>
		//prepare for next iteration
		row++;
		ss_.str(string());
		ss_ << row;
		parsed_data_set = NULL;
	}
	//************************************************************************************************ <set><var>xxx</var></set>
	return true;
}

