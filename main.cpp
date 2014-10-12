#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "Serialization\SerializeCSV.h"

//using namespace cv;
//using namespace std;


int main( int argc, char** argv )
{
	cv::Mat image = cv::imread("C:\\Users\\Grey\\Documents\\DEV\\kinectScanner\\TestData\\boxes1.bmp", CV_LOAD_IMAGE_GRAYSCALE);
	
	cv::imshow("image test", image);


	//Thread ^thr1 = gcnew Thread(gcnew ThreadStart());

	char endOfTheLine;
	std::cout << "Terminated!";
	std::cin >> endOfTheLine;

    return 0;
}