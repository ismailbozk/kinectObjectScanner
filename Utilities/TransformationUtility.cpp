#include "TransformationUtility.h"

#include <opencv2/legacy/legacy.hpp>

#include "../Macros.h"
#include <iostream>

using namespace std;
using namespace cv;

std::vector<Match3D> *TransformationUtility::Create3DMatchPoints(std::vector<bool> &mask, std::vector<std::vector<cv::DMatch>> &matches, BaseKinectModel &trainKinectModel, std::vector<cv::KeyPoint> &trainKeypoints, BaseKinectModel &testKinectModel, std::vector<cv::KeyPoint> &testKeypoints)
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

			if ((trainKinectModel.pointCloud[trainPairIndex].x == 0.0 && trainKinectModel.pointCloud[trainPairIndex].y == 0.0 && trainKinectModel.pointCloud[trainPairIndex].z == 0.0)
				|| (testKinectModel.pointCloud[testPairIndex].x == 0.0 && testKinectModel.pointCloud[testPairIndex].y == 0.0 && testKinectModel.pointCloud[testPairIndex].z == 0.0))
			{///elininate zero Point3f's
				mask[i] = false;
			}
			else
			{
				Match3D match3d = Match3D(testKinectModel.pointCloud[testPairIndex], trainKinectModel.pointCloud[trainPairIndex]);
				(*matches3D).push_back(match3d);
			}
		}
	}

	return matches3D;
}


cv::Matx44f *TransformationUtility::CreateTransformation(std::vector<Match3D> &matches3D)
{
	cv::Matx44f *transformation = new cv::Matx44f();

#pragma region obtain mid points of matches and seperated consecutive matches
	Point3f trainMidPoint = Point3f(0,0,0);		//d_ mid
	Point3f testMidPoint = Point3f(0,0,0);		//m_ mid
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
	float HMatrix11 = 0.0;
    float HMatrix12 = 0.0;
    float HMatrix13 = 0.0;
    float HMatrix21 = 0.0;
    float HMatrix22 = 0.0;
    float HMatrix23 = 0.0;
    float HMatrix31 = 0.0;
    float HMatrix32 = 0.0;
    float HMatrix33 = 0.0;	

	vector<Match3D> matches3DWhichAreTranslatedAroundTheMidPoints = matches3D;

	for (int i = 0; i <matches3DWhichAreTranslatedAroundTheMidPoints.size(); i++)
	{
		matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair -= trainMidPoint;		//d_i - d_ = d_c_i
		matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair -= testMidPoint;		//m_i - m_ = m_c_i

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
	cv::Matx33f src = cv::Matx33f();

	src(0,0) = HMatrix11; src(0,1) = HMatrix12; src(0,2) = HMatrix13;
	src(1,0) = HMatrix21; src(1,1) = HMatrix22; src(1,2) = HMatrix23;
	src(2,0) = HMatrix31; src(2,1) = HMatrix32; src(2,2) = HMatrix33;

	cv::Matx31f w;
	cv::Matx33f u;
	cv::Matx33f vt;
	cv::SVD::compute(src, w, u, vt, 0);
#pragma endregion

#pragma region Rotation Matrix
	/*(*transformation)(0,0) = vt(0,0) * u(0,0) + vt(1,0) * u(0,1) + vt(2,0) * u(0,2);
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
	(*transformation)(2,3) = 0.0;*/

	Matx33f uMultipleVT = u*vt;
	(*transformation)(0,0) = uMultipleVT(0,0);
	(*transformation)(0,1) = uMultipleVT(0,1);
	(*transformation)(0,2) = uMultipleVT(0,2);
	(*transformation)(0,3) = 0.0;

	(*transformation)(1,0) = uMultipleVT(1,0);
	(*transformation)(1,1) = uMultipleVT(1,1);
	(*transformation)(1,2) = uMultipleVT(1,2);
	(*transformation)(1,3) = 0.0;

	(*transformation)(2,0) = uMultipleVT(2,0);
	(*transformation)(2,1) = uMultipleVT(2,1);
	(*transformation)(2,2) = uMultipleVT(2,2);
	(*transformation)(2,3) = 0.0;

	(*transformation)(3,0) = 0.0;
	(*transformation)(3,1) = 0.0;
	(*transformation)(3,2) = 0.0;
	(*transformation)(3,3) = 1.0;

#pragma endregion

#pragma region camera translation
	cv::Matx14f translation;

	cv::Matx14f midTestPointMat = cv::Matx14f();
	midTestPointMat(0,0) = testMidPoint.x;	midTestPointMat(0,1) = testMidPoint.y;
	midTestPointMat(0,2) = testMidPoint.z;	midTestPointMat(0,3) = 1.0;
	
	cv::Matx14f midTrainPointMat = cv::Matx14f();
	midTrainPointMat(0,0) = trainMidPoint.x;	midTrainPointMat(0,1) = trainMidPoint.y;
	midTrainPointMat(0,2) = trainMidPoint.z;	midTrainPointMat(0,3) = 1.0;

	TransformationUtility::TransformSinglePoint(midTestPointMat, (*transformation));
    translation = midTrainPointMat - midTestPointMat;

	(*transformation)(3,0) = translation(0,0);
	(*transformation)(3,1) = translation(0,1);
	(*transformation)(3,2) = translation(0,2);
	(*transformation)(3,3) = 1.0;
#pragma endregion

	return transformation;
}



std::vector<SCPoint3D> *TransformationUtility::Transform(BaseKinectModel &testModel, cv::Matx44f &transformationMatrix)
{
	std::vector<SCPoint3D> *result = new std::vector<SCPoint3D>();
	(*result).reserve(testModel.pointCloud.size());

	cv::Mat grayImageWith3Channels;
	if (testModel.image.cols == 0)
	{//if no image found then use grayImage
		cvtColor(testModel.grayImage, grayImageWith3Channels, CV_GRAY2RGB);
	}

	for (int i = 0; i < testModel.pointCloud.size(); i++)
	{
		if (testModel.pointCloud[i].z != 0.0 && testModel.pointCloud[i].y != 0.0 && testModel.pointCloud[i].x != 0.0)
		{//if point is valid
			if (testModel.image.cols > 0)
			{//if color image exists
				(*result).push_back(SCPoint3D(TransformationUtility::TransformSinglePoint(testModel.pointCloud[i], transformationMatrix),
					testModel.image.at<cv::Vec3b>(i / testModel.image.cols, i % testModel.image.cols)));
			}
			else if (testModel.grayImage.cols > 0)
			{//if gray color image exists
				(*result).push_back(SCPoint3D(TransformationUtility::TransformSinglePoint(testModel.pointCloud[i], transformationMatrix),
					grayImageWith3Channels.at<cv::Vec3b>(i / grayImageWith3Channels.cols, i % grayImageWith3Channels.cols)));
			}
			else
			{// if no image fount than use white as color
				cv::Vec3b white(255, 255, 255);
				(*result).push_back(SCPoint3D(TransformationUtility::TransformSinglePoint(testModel.pointCloud[i], transformationMatrix),
					cv::Vec3b(255, 255, 255)));
			}
		}
	}

	return result;
}

std::vector<Match3D> *TransformationUtility::RANSAC(std::vector<Match3D> &matches3D, int numberOfIteration, float threshold)
{//requeiress min 3 Match3D
	std::vector<unsigned int> bestMatchesIndices;
	int cBest = INT_MIN;

	for (int iteration = 0; iteration < numberOfIteration; iteration++)
	{
		int c = 0;

		std::vector<unsigned int> *random3 = TransformationUtility::Generate3UniqueRandom(matches3D.size());

		std::vector<Match3D> pickedMatches3D = std::vector<Match3D>();
		pickedMatches3D.reserve(3);
		pickedMatches3D.push_back(matches3D[(*random3)[0]]);
		pickedMatches3D.push_back(matches3D[(*random3)[1]]);
		pickedMatches3D.push_back(matches3D[(*random3)[2]]);

		cv::Matx44f *candTransformation = TransformationUtility::CreateTransformation(pickedMatches3D);
		std::vector<unsigned int> candMatchesIndices = std::vector<unsigned int>();
		candMatchesIndices.reserve(matches3D.size());

		float euclideanDistance;

		for (unsigned int matchIndex = 0; matchIndex < matches3D.size(); matchIndex++)
		{
			TransformationUtility::EuclideanDistanceBetweenTwoPoint(euclideanDistance,
				matches3D[matchIndex].trainPair, 
				TransformationUtility::TransformSinglePoint(matches3D[matchIndex].queryPair, *candTransformation));

			if (euclideanDistance < threshold)
			{
				c++;
				candMatchesIndices.push_back(matchIndex);
			}
		}

		if (c > cBest && TransformationUtility::IsTransformationMatrixRightHanded(*candTransformation))
		{
			cBest = c;
			bestMatchesIndices = candMatchesIndices;
		}

		//clear heap
		delete candTransformation;
		delete random3;
	}

	std::vector<Match3D> *result = new std::vector<Match3D>(bestMatchesIndices.size());

	for (int i = 0; i < bestMatchesIndices.size(); i++)
	{
		(*result).push_back(matches3D[bestMatchesIndices[i]]);
	}

	return result;
}


#pragma region Helpers

cv::Point3f TransformationUtility::TransformSinglePoint(cv::Point3f &pt, cv::Matx44f &transformationMatrix)
{
	Matx44f transformationT;
	cv::transpose(transformationMatrix, transformationT);

	cv::Matx41f point(pt.x, pt.y, pt.z, 1.0);
	cv::Matx41f result = transformationT * point;
	return cv::Point3f(result(0,0), result(1,0), result(2.0));
}

void TransformationUtility::TransformSinglePoint(cv::Matx41f &pt, cv::Matx44f &transformationMatrix)
{
	Matx44f transformationT;
	cv::transpose(transformationMatrix, transformationT);
	pt = transformationT * pt;	
}

void TransformationUtility::TransformSinglePoint(cv::Matx14f &pt, cv::Matx44f &transformationMatrix)
{
	pt = pt * transformationMatrix;
}

std::vector<unsigned int> *TransformationUtility::Generate3UniqueRandom(unsigned int ceil)
{
	std::vector<unsigned int> *uniqueNumbers = new std::vector<unsigned int>(3);

	if (ceil >= 3)
	{
		(*uniqueNumbers)[0] = (std::rand() % ceil);

		(*uniqueNumbers)[1] = (std::rand() % (ceil - 1));
		if ((*uniqueNumbers)[0] == (*uniqueNumbers)[1])
		{
			(*uniqueNumbers)[1] += 1;
		}

		(*uniqueNumbers)[2] = (std::rand() % (ceil - 2));
		if ((*uniqueNumbers)[2] == (*uniqueNumbers)[0])
		{
			(*uniqueNumbers)[2] += 1;
		}
		if ((*uniqueNumbers)[2] == (*uniqueNumbers)[1])
		{
			(*uniqueNumbers)[2] =+ 1;
		}
	}
	return uniqueNumbers;
}

bool TransformationUtility::IsTransformationMatrixRightHanded(cv::Matx44f &transformation)
{
	float determinant = cv::determinant(transformation);
	if (determinant > 0.0)
	{
		return true;
	}
	return false;
}

void TransformationUtility::EuclideanDistanceBetweenTwoPoint(float &euclideanDistance, cv::Point3f &pointA, cv::Point3f &pointB)
{
	euclideanDistance = cv::norm(pointA - pointB);
}

#pragma endregion