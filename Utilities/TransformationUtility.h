#ifndef TRANSFORMATIONUTILITY_H
#define TRANSFORMATIONUTILITY_H

#include <opencv2/nonfree/nonfree.hpp>

#include <vector>

#include "../Models/BaseKinectModel.h"
#include "../Models/Match3D.h"
#include "../Models/SCPoint3D.h"


class TransformationUtility
{
public:
	static std::vector<Match3D> *TransformationUtility::Create3DMatchPoints(std::vector<bool> &mask, std::vector<std::vector<cv::DMatch>> &matches, BaseKinectModel &trainKinectModel, std::vector<cv::KeyPoint> &trainKeypoints, BaseKinectModel &testKinectModel, std::vector<cv::KeyPoint> &testKeypoints);
	static cv::Matx44f *TransformationUtility::CreateTransformation(std::vector<Match3D> &matches3D);

	static std::vector<SCPoint3D> *TransformationUtility::Transform(BaseKinectModel &testModel, cv::Matx44f &transformationMatrix);

	static std::vector<Match3D> *TransformationUtility::RANSAC(std::vector<Match3D> &matches3D, int numberOfIteration, float threshold);

	static cv::Point3f TransformationUtility::TransformSinglePoint(cv::Point3f &pt, cv::Matx44f &transformationMatrix);
	static void TransformationUtility::TransformSinglePoint(cv::Matx41f &pt, cv::Matx44f &transformationMatrix);
	static void TransformationUtility::TransformSinglePoint(cv::Matx14f &pt, cv::Matx44f &transformationMatrix);

private:
	static std::vector<unsigned int> *TransformationUtility::Generate3UniqueRandom(unsigned int ceil);
	static bool TransformationUtility::IsTransformationMatrixRightHanded(cv::Matx44f &transformation);
	static void TransformationUtility::EuclideanDistanceBetweenTwoPoint(float &euclideanDistance, cv::Point3f &pointA, cv::Point3f &pointB);
};

#endif

