#ifndef TRANSFORMATIONUTILITY_H
#define TRANSFORMATIONUTILITY_H

//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
//#include <opencv2/legacy/legacy.hpp>
#include "..\Models\BaseKinectModel.h"
#include "../Models/Match3D.h"
#include <vector>

class TransformationUtility
{
public:

	static std::vector<Match3D> TransformationUtility::Create3DMatchPoints(std::vector<bool> &mask, std::vector<std::vector<cv::DMatch>> &matches, BaseKinectModel &trainKinectModel, std::vector<cv::KeyPoint> &trainKeypoints, BaseKinectModel &testKinectModel, std::vector<cv::KeyPoint> &testKeypoints);
	static cv::Mat4f TransformationUtility::CreateTransformation(std::vector<Match3D> &matches3D);

private:

};

#endif

