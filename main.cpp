#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "Serialization\SerializationUtility.h"
#include <thread>
#include <process.h>

#include <boost/lambda/lambda.hpp>
#include <iterator>
#include <algorithm>

#include <boost/thread.hpp>
#include <boost/date_time.hpp>

using namespace cv;
using namespace std;

void workerFunc()
{
	boost::posix_time::seconds workTime(3);

	std::cout << "Worker: running" << std::endl;

	boost::this_thread::sleep(workTime);

	std::cout << "Worker: Finished" << std::endl;
}

int main( int argc, char** argv )
{
	std::cout << "main: startup" << std::endl;

	boost::thread workerThread(workerFunc);

	std::cout << "main: waiting for thread" << std::endl;

	workerThread.join();

	std::cout << "main: done" << endl;


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