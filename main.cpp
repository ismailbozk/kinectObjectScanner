#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "Serialization\SerializationUtility.h"
#include <thread>
#include <process.h>

#include <boost/lambda/lambda.hpp>
#include <iterator>
#include <algorithm>

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
	using namespace boost::lambda;
    typedef std::istream_iterator<int> in;

    std::for_each(
        in(std::cin), in(), std::cout << (_1 * 3) << " " );


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