#include "SerializationUtility.h"

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

void deserializeKinectOutputs(string filePrefix, short[]& depthData)
{

}

private short[] getDepth(string filePath)
{
	string line;
	ifstream depthFile(filePath);
	if (depthFile.is_open())
	{
		while(getline(depthFile, line))
		{
			cout << line << '\n';
		}
	}
	else
	{
		cout << "Unable to open file"; 
	}
}
