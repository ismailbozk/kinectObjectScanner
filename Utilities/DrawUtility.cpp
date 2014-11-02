#include "DrawUtility.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <stdexcept>

using namespace std;
using namespace cv;

static Scalar drawColor = Scalar(0, 255, 0, 1);

void DrawUtility::DrawMatches(std::vector<bool> mask, std::vector<std::vector<cv::DMatch>> matches, cv::Mat queryImage, std::vector<cv::KeyPoint> queryKeyPoints, cv::Mat trainImage, std::vector<cv::KeyPoint> trainKeyPoints)
{
	if (mask.size() == matches.size() && mask.size() == queryKeyPoints.size())
	{
		//if image is GrayScale image  then convert to rgb image
		if (queryImage.channels() != 3)	//color Image
		{
			cv::Mat queryImageRGB(queryImage.size(), CV_8UC3);

			// convert grayscale to color image
			cv::cvtColor(queryImage, queryImageRGB, CV_GRAY2RGB);
			queryImage = queryImageRGB;
		}
		if (trainImage.channels() != 3) //color image
		{
			cv::Mat trainImageRGB(trainImage.size(), CV_8UC3);

			// convert grayscale to color image
			cv::cvtColor(trainImage, trainImageRGB, CV_GRAY2RGB);
			trainImage = trainImageRGB;
		}

		//prepare match result image
		Size sz1 = queryImage.size();
		Size sz2 = trainImage.size();
		Mat imgMatches(sz1.height, sz1.width+sz2.width, queryImage.type());
		// Move right boundary to the left.
		imgMatches.adjustROI(0, 0, 0, -sz2.width);
		queryImage.copyTo(imgMatches);
		imgMatches.adjustROI(0, 0, 0, sz1.width+sz2.width);
		// Move the left boundary to the right, right boundary to the right.
		imgMatches.adjustROI(0, 0, -sz1.width, 0);
		trainImage.copyTo(imgMatches);
		imgMatches.adjustROI(0, 0, sz1.width, 0);

		for (int i = 0; i < mask.size(); i++)
		{
			if (mask[i] && matches[i].size() > 0)
			{
				DMatch currentMatch = matches[i][0];
				KeyPoint queryKeyPoint = queryKeyPoints[currentMatch.queryIdx];
				KeyPoint trainKeyPoint = trainKeyPoints[currentMatch.trainIdx];

				//1. draw query image feature circle
				circle(imgMatches, queryKeyPoint.pt, (queryKeyPoint.size / 2), drawColor, 1, 8, 0);
				//2. Draw train image feature circle
				Point2f trainFeaturePoint = trainKeyPoint.pt;
				trainFeaturePoint.x = trainFeaturePoint.x + queryImage.size().width;
				circle(imgMatches, trainFeaturePoint, (trainKeyPoint.size / 2), drawColor, 1, 8, 0);
				//3. Finally line two circle
				line(imgMatches, queryKeyPoint.pt, trainFeaturePoint, drawColor, 1, 8, 0);
			}
		}

		imshow("Matches of Consecutive Frames", imgMatches);
		waitKey(0);
	}
	else
	{
		string errorMessage = "DrawUtility::DrawMatches failed at \"if (mask.size() == matches.size() && mask.size() == queryKeyPoints.size())\"";
		cout << errorMessage << endl;
		throw std::invalid_argument(errorMessage );
	}
}
