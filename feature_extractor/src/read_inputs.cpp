#include <fstream>
#include <iostream>
#include <map>
#include <string>


#include "read_inputs.h"


using namespace std;
// read the parameter's file
bool read_inputs (map<string,float> &parameters)
{
	ifstream file("../parameters_file");

	if (! file.is_open())
	{
		cout<< "Could not open parameters file"<< endl;
		return false;
	}

	string name;
	float value;

	while (!file.eof())
	{
		file >> name;
		file >> parameters[name];

	}

	cout<< "Finish reading parameters file"<< endl;
	return true;

}



