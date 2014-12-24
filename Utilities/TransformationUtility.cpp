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


cv::Matx44d TransformationUtility::CreateTransformation(std::vector<Match3D> &matches3D)
{
	cv::Matx44d *transformation = new cv::Matx44d();

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

#pragma endregion

#pragma region SVD
	cv::Matx33d src = cv::Matx33d();

	src(0,0) = HMatrix11; src(0,1) = HMatrix12; src(0,2) = HMatrix13;
	src(1,0) = HMatrix21; src(1,1) = HMatrix22; src(1,2) = HMatrix23;
	src(2,0) = HMatrix31; src(2,1) = HMatrix32; src(2,2) = HMatrix33;

	cv::Matx31d w;
	cv::Matx33d u;
	cv::Matx33d vt;
	cv::SVD::compute(src, w, u, vt, 0);
#pragma endregion

#pragma region Rotation Matrix
	(*transformation)(0,0) = vt(0,0) * u(0,0) + vt(1,0) * u(0,1) + vt(2,0) * u(0,2);
	(*transformation)(0,1) = vt(0,0) * u(1,0) + vt(1,0) * u(1,1) + vt(2,0) * u(1,2);
	(*transformation)(0,2) = vt(0,0) * u(2,0) + vt(1,0) * u(2,1) + vt(2,0) * u(2,2);
	(*transformation)(0,3) = 0.0;

	(*transformation)(1,0) = vt(0,1) * u(0,0) + vt(1,1) * u(0,1) + vt(2,1) * u(0,2);
	(*transformation)(1,1) = vt(0,1) * u(1,0) + vt(1,1) * u(1,1) + vt(2,1) * u(1,2);
	(*transformation)(1,2) = vt(0,1) * u(2,0) + vt(1,1) * u(2,1) + vt(2,1) * u(2,2);
	(*transformation)(1,3) = 0.0;

	(*transformation)(2,0) = vt(0,2) * u(0,0) + vt(1,2) * u(0,1) + vt(2,2) * u(0,2);
	(*transformation)(2,1) = vt(0,2) * u(1,0) + vt(1,2) * u(1,1) + vt(2,2) * u(1,2);
	(*transformation)(2,2) = vt(0,2) * u(2,0) + vt(1,2) * u(2,1) + vt(2,2) * u(2,2);
	(*transformation)(2,3) = 0.0;

	(*transformation)(2,0) = 0.0;
	(*transformation)(2,1) = 0.0;
	(*transformation)(2,2) = 0.0;
	(*transformation)(2,3) = 1.0;

#pragma endregion

#pragma region camera translation
	cv::Matx41d translation;

	cv::Matx41d midTestPointMat = cv::Matx41d();
	midTestPointMat(0,0) = testMidPoint.x;	midTestPointMat(1,0) = testMidPoint.y;
	midTestPointMat(2,0) = testMidPoint.z;	midTestPointMat(3,0) = 1.0;
	
	cv::Matx41d midTrainPointMat = cv::Matx41d();
	midTrainPointMat(0,0) = trainMidPoint.x;	midTrainPointMat(1,0) = trainMidPoint.y;
	midTrainPointMat(2,0) = trainMidPoint.z;	midTrainPointMat(3,0) = 1.0;

	testMidPoint = TransformationUtility::TransformSinglePoint(testMidPoint, *transformation);

	//cv::transform(midTestPointMat, midTestPointMat, *transformation);

    translation = midTrainPointMat - midTestPointMat;

	(*transformation)(3,0) = translation(0,0);
	(*transformation)(3,0) = translation(1,0);
	(*transformation)(3,0) = translation(2,0);
	(*transformation)(3,0) = 1.0;
#pragma endregion

	return *transformation;
}

//std::vector<SCPoint3D> TransformationUtility::Transform(BaseKinectModel kinectModel, cv::Mat &transformationMatrix)
//{
//	cv::Point3d
//}

cv::Point3d TransformationUtility::TransformSinglePoint(cv::Point3d &pt, cv::Matx44d &transformationMatrix)
{
	cv::Matx41d point(pt.x, pt.y, pt.z, 1.0);
	cv::Matx41d result = transformationMatrix * point;
	return cv::Point3d(result(0,0), result(1,0), result(2.0));
}