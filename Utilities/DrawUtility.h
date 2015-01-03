#ifndef DRAWUTILITY_H
#define DRAWUTILITY_H

#include <vector>
#include <opencv2/core/core.hpp>
#include "opencv2/nonfree/features2d.hpp"

#include "../Models/SCPoint3D.h"

class DrawUtility
{
public:
	static void DrawUtility::DrawMatches(std::vector<bool> mask, std::vector<std::vector<cv::DMatch>> matches, cv::Mat queryImage, std::vector<cv::KeyPoint> queryKeyPoints, cv::Mat trainImage, std::vector<cv::KeyPoint> trainKeyPoints);
	static void DrawUtility::WritePLYFile(std::string fileName, std::vector<SCPoint3D> pointCloud);
};
#endif