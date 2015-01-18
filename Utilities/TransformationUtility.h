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
	static cv::Matx44d *TransformationUtility::CreateTransformation(std::vector<Match3D> &matches3D);

	static std::vector<SCPoint3D> *TransformationUtility::Transform(BaseKinectModel &testModel, cv::Matx44d &transformationMatrix);

	static std::vector<Match3D> *TransformationUtility::RANSAC(std::vector<Match3D> &matches3D, int numberOfIteration, double threshold);

	static cv::Point3d TransformationUtility::TransformSinglePoint(cv::Point3d &pt, cv::Matx44d &transformationMatrix);
	static void TransformationUtility::TransformSinglePoint(cv::Matx41d &pt, cv::Matx44d &transformationMatrix);

private:
	static std::vector<unsigned int> *TransformationUtility::Generate3UniqueRandom(unsigned int ceil);
	static bool TransformationUtility::IsTransformationMatrixRightHanded(cv::Matx44d &transformation);
	static void TransformationUtility::EuclideanDistanceBetweenTwoPoint(double &euclideanDistance, cv::Point3d &pointA, cv::Point3d &pointB);
};

#endif

