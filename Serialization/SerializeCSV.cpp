#include "SerializeCSV.h"

#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace System;
using namespace System::IO;
using namespace System::Collections;
using namespace System::Runtime::Serialization::Formatters::Binary;
using namespace System::Runtime::Serialization;

static const string testDataPath = "C:\\Users\\Grey\\Documents\\DEV\\kinectScanner\\TestData\\";

void SerializeCSV::serialize()
	{
		string line;
		string path = (testDataPath + "boxes1Depth.csv");
		ifstream depthFile(path);
		if (depthFile.is_open())
		{
			while(getline(depthFile, line))
			{
				//cout << line << '\n';
			}
		}
		else
		{
			cout << "Unable to open file"; 
		}

	}