#include "TransformationUtility.h"
#include "../Macros.h"
#include <iostream>

using namespace std;
using namespace cv;

std::vector<Match3D> TransformationUtility::Create3DMatchPoints(std::vector<bool> &mask, std::vector<std::vector<cv::DMatch>> &matches, BaseKinectModel &trainKinectModel, std::vector<cv::KeyPoint> &trainKeypoints, BaseKinectModel &testKinectModel, std::vector<cv::KeyPoint> &testKeypoints)
{
	vector<Match3D> *matches3D = new vector<Match3D>();
	for (int i = 0; i < mask.size(); i++)
	{
		if (mask[i] == true)
		{
			DMatch currentMatch = matches[i][0];
			KeyPoint testPairKeyPoint = testKeypoints[currentMatch.queryIdx];
			KeyPoint trainPairKeyPoint = trainKeypoints[currentMatch.trainIdx];

			int trainPairIndex = (int)(((trainKinectModel.grayImage.cols) * round(trainPairKeyPoint.pt.y)) + (int)trainPairKeyPoint.pt.x);

			int testPairIndex = (int)(((testKinectModel.grayImage.cols) * round(testPairKeyPoint.pt.y)) + (int)testPairKeyPoint.pt.x);
			cout << "" << trainPairIndex << " - " << testPairIndex << endl;

			if (trainPairIndex == 163484)
			{
				bool c = false;
				c= true;
			}

			if (trainPairIndex < trainKinectModel.grayImage.cols * trainKinectModel.grayImage.rows &&
				testPairIndex < testKinectModel.grayImage.cols * testKinectModel.grayImage.rows)
			{
				Match3D match3d = Match3D(trainKinectModel.pointCloud[trainPairIndex], testKinectModel.pointCloud[testPairIndex]);
				(*matches3D).push_back(match3d);
			}
			else
			{
				cout << "trainX" << trainPairKeyPoint.pt.x << "trainY" << round(trainPairKeyPoint.pt.y) << endl;
				cout << "testX" << testPairKeyPoint.pt.x << "testY" << round(testPairKeyPoint.pt.y) << endl;
			}
		}
	}

	return *matches3D;
}


cv::Mat4f TransformationUtility::CreateTransformation(std::vector<std::vector<cv::DMatch>> &matches )
{
	cv::Mat4f *transformation = new cv::Mat4f();
	cv::Vec3f translation();
	
	//obtain mid points of matches and seperated consecutive matches




	return *transformation;
}
