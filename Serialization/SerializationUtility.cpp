#include "SerializationUtility.h"

#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace System;
using namespace System::IO;

static const string testDataPath = "C:\\Users\\Grey\\Documents\\DEV\\kinectScanner\\TestData\\";

void deserializeKinectOutputs(string filePrefix, short[]& depthData)
{

}

private short[] getDepth(string filePath)
{
	std::string line;
	std::ifstream depthFile(filePath);
	std::stringstream ss;

	if (depthFile.is_open())
	{
		while(getline(depthFile, line))
		{
			;
		}
	}
	else
	{
		cout << "Unable to open file"; 
	}
}
