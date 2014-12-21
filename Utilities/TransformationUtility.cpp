#include "TransformationUtility.h"

#include <opencv2/core/core.hpp>

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


			if (trainKinectModel.pointCloud[trainPairIndex] == cv::Point3d() || testKinectModel.pointCloud[testPairIndex] == cv::Point3d())
			{///elininate zero Point3d's
				mask[i] = false;
			}
			else
			{
				Match3D match3d = Match3D(trainKinectModel.pointCloud[trainPairIndex], testKinectModel.pointCloud[testPairIndex]);
				(*matches3D).push_back(match3d);
			}
		}
	}

	return *matches3D;
}


cv::Mat4f TransformationUtility::CreateTransformation(std::vector<Match3D> &matches3D)
{
	cv::Mat4f *transformation = new cv::Mat4f();
	cv::Vec3f translation();

	#pragma region obtain mid points of matches and seperated consecutive matches
	Point3d trainMidPoint = Point3d(0,0,0);		//d_ mid
	Point3d testMidPoint = Point3d(0,0,0);		//m_ mid
	for (int i = 0; i < matches3D.size(); i++)
	{
		trainMidPoint += matches3D[i].trainPair;
		testMidPoint += matches3D[i].queryPair;
	}
	trainMidPoint.x = trainMidPoint.x / matches3D.size();
	trainMidPoint.y = trainMidPoint.y / matches3D.size();
	trainMidPoint.z = trainMidPoint.z / matches3D.size();
	testMidPoint.x = testMidPoint.x / matches3D.size();
	testMidPoint.y = testMidPoint.y / matches3D.size();
	testMidPoint.z = testMidPoint.z / matches3D.size();
	#pragma endregion

	#pragma region pull the all points to around origin midpoints traslated to the 0,0,0 point and finding the H matrix
	double HMatrix11 = 0.0;
    double HMatrix12 = 0.0;
    double HMatrix13 = 0.0;
    double HMatrix21 = 0.0;
    double HMatrix22 = 0.0;
    double HMatrix23 = 0.0;
    double HMatrix31 = 0.0;
    double HMatrix32 = 0.0;
    double HMatrix33 = 0.0;	

	vector<Match3D> matches3DWhichAreTranslatedAroundTheMidPoints = matches3D;

	for (int i = 0; i <matches3DWhichAreTranslatedAroundTheMidPoints.size(); i++)
	{
		matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair -= trainMidPoint;		//d_i - d_ = d_c_i
		matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair -= testMidPoint;		//m_i - m_ = m_c_i

		HMatrix11 += matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair.x * matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair.x;
		HMatrix12 += matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair.x * matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair.y;
		HMatrix13 += matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair.x * matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair.z;

		HMatrix21 += matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair.y * matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair.x;
		HMatrix22 += matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair.y * matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair.y;
		HMatrix23 += matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair.y * matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair.z;

		HMatrix31 += matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair.z * matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair.x;
		HMatrix32 += matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair.z * matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair.y;
		HMatrix33 += matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair.z * matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair.z;
	}
	cv::Mat src = cv::Mat(3, 3, CV_64F);
	
	src.at<double>(0,0) = HMatrix11; src.at<double>(0,1) = HMatrix12; src.at<double>(0,2) = HMatrix13;
	src.at<double>(1,0) = HMatrix21; src.at<double>(1,1) = HMatrix22; src.at<double>(1,2) = HMatrix23;
	src.at<double>(2,0) = HMatrix31; src.at<double>(2,1) = HMatrix32; src.at<double>(2,2) = HMatrix33;

	cv::Mat w;
	cv::Mat u;
	cv::Mat vt;
	cv::SVD::compute(src, w, u, vt, 0);

	

	#pragma endregion

	return *transformation;
}
