#include "SerializeCSV.h"

#include <iostream>
#include <fstream>
#include <string>

//using namespace std;
//using namespace System;
//using namespace System::IO;

const static std::string testDataPath = "C:\\Users\\Grey\\Documents\\DEV\\kinectScanner\\TestData\\";

void SerializeCSV::serialize()
	{
		std::string line;
		std::string path = (testDataPath + "boxes1Depth.csv");
		std::ifstream depthFile(path);
		if (depthFile.is_open())
		{
			while(getline(depthFile, line))
			{
				std::cout << line << '\n';
			}
		}
		else
		{
			std::cout << "Unable to open file"; 
		}

	}
