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

	static std::vector<Match3D> TransformationUtility::Create3DMatchPoints(std::vector<bool> &mask, std::vector<std::vector<cv::DMatch>> &matches, BaseKinectModel &trainKinectModel, std::vector<cv::KeyPoint> &trainKeypoints, BaseKinectModel &testKinectModel, std::vector<cv::KeyPoint> &testKeypoints);
	static cv::Matx44d TransformationUtility::CreateTransformation(std::vector<Match3D> &matches3D);

	static std::vector<SCPoint3D> TransformationUtility::Transform(BaseKinectModel kinectModel, cv::Mat &transformationMatrix);

	static cv::Point3d TransformSinglePoint(cv::Point3d &pt, cv::Matx44d &transformationMatrix);
;

private:


};

#endif

