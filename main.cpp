#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "Serialization\SerializationUtility.h"
#include <thread>
#include <process.h>

using namespace cv;
using namespace std;


int main( int argc, char** argv )
{
	cv::Mat image = SerializationUtility::getGrayScaleImage("boxes1");
	std::vector<short> depth = SerializationUtility::getDepthData("boxes1");
	std::vector<cv::Vec3b> color = SerializationUtility::getColorData("boxes1");

	char endOfTheLine;
	std::cout << "Terminated!";
	std::cin >> endOfTheLine;
	
    return 0;
}

//void testbla()
//{
//	cv::Mat image = SerializationUtility::getGrayScaleImage("boxes1");
//
//	cv::imshow("image test", image);
//	cv::waitKey(1000);
//}