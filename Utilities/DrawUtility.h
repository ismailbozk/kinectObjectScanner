#ifndef DRAWUTILITY_H
#define DRAWUTILITY_H

#include <vector>
#include <opencv2/core/core.hpp>
#include "opencv2/nonfree/features2d.hpp"

class DrawUtility
{
public:
	static void DrawUtility::DrawMatches(std::vector<bool> mask, std::vector<std::vector<cv::DMatch>> matches, cv::Mat queryImage, std::vector<cv::KeyPoint> queryKeyPoints, cv::Mat trainImage, std::vector<cv::KeyPoint> trainKeyPoints);
};
#endif