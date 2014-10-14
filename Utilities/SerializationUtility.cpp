#include "SerializationUtility.h"

#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

typedef unsigned char BYTE;

using namespace std;

static const std::string testDataPath = "C:\\Users\\Grey\\Documents\\DEV\\kinectScanner\\TestData\\";
static const std::string testGrayImageExtension = ".bmp";
static const std::string testDepthDataPostfix = "Depth.csv";
static const std::string testRGBDataPostfix = "RGB.csv";

cv::Mat SerializationUtility::getGrayScaleImage(std::string filePrefix)
{
	return cv::imread(testDataPath + filePrefix + testGrayImageExtension, CV_LOAD_IMAGE_UNCHANGED);
}

std::vector<short> SerializationUtility::getDepthData(std::string filePrefix)
{
	std::vector<short> depthData;

	std::string line;
	std::ifstream depthFile(testDataPath + filePrefix + testDepthDataPostfix);

	if (depthFile.is_open())
	{
		while(getline(depthFile, line))
		{
			short num = atoi(line.c_str());
			depthData.push_back(num);
		}
	}
	return depthData;
}

std::vector<cv::Vec3b> SerializationUtility::getColorData(std::string filePrefix)
{
	std::vector<cv::Vec3b> colorData;

	std::string line;
	std::ifstream colorFile(testDataPath + filePrefix + testRGBDataPostfix);

	if (colorFile.is_open())
	{
		while(getline(colorFile, line))
		{
			std::stringstream ss(line);
			string token;

			std::getline(ss, token, ';');
			//std::cout << token << '\n';
			uchar r = atoi(token.c_str());
			std::getline(ss, token, ';');
			//std::cout << token << '\n';
			uchar g = atoi(token.c_str());
			std::getline(ss, token, ';');
			//std::cout << token << '\n';
			uchar b = atoi(token.c_str());

			cv::Vec3b color = cv::Vec3b(r, g, b);

			colorData.push_back(color);
		}
	}
	return colorData;
}

/* Option 1
#include <fstream>
#include <vector>
typedef unsigned char BYTE;

std::vector<BYTE> readFile(const char* filename)
{
    // open the file:
    std::streampos fileSize;
    std::ifstream file(filename, std::ios::binary);

    // get its size:
    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    // read the data:
    std::vector<BYTE> fileData(fileSize);
    file.read((char*) &fileData[0], fileSize);
    return fileData;
}
*/

/* Option 2
std::vector<BYTE> readFile(const char* filename)
{
    // open the file:
    std::ifstream file(filename, std::ios::binary);

    // read the data:
    return std::vector<BYTE>((std::istreambuf_iterator<char>(file)),
                              std::istreambuf_iterator<char>());
}
*/

/* Option3
std::vector<BYTE> readFile(const char* filename)
{
    // open the file:
    std::basic_ifstream<BYTE> file(filename, std::ios::binary);

    // read the data:
    return std::vector<BYTE>((std::istreambuf_iterator<BYTE>(file)),
                              std::istreambuf_iterator<BYTE>());
}
*/