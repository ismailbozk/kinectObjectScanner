#ifndef FEATURES2DUTILITY_H
#define FEATURES2DUTILITY_H

#include <vector>
#include <opencv2/core/core.hpp>
#include "opencv2/nonfree/features2d.hpp"
#include "..\Models\BaseKinectModel.h"
#include "..\Models\DepthScale.h"

class Features2DUtility
{
public:
	static void Features2DUtility::VoteForUniqueness(std::vector<std::vector<cv::DMatch>>& matches, float threshold, std::vector<bool> &mask);
	static std::vector<DepthScale> Features2DUtility::CreateInlierDepthScales(BaseKinectModel &kinectModel, std::vector<cv::KeyPoint> &keyPoints);
};
#endif
