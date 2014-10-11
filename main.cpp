#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "Serialization\SerializeCSV.h"

using namespace cv;
using namespace std;

int main( int argc, char** argv )
{
	Mat testImage = imread("TestData//boxes1.bmp", CV_LOAD_IMAGE_GRAYSCALE);

	SerializeCSV::serialize();


    return 0;
}