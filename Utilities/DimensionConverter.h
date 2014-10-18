#include <opencv2/core/core.hpp>

#ifndef DIMENSIONCONVERTER_H
#define DIMENSIONCONVERTER_H

class DimensionConverter
{
public:
	static cv::Point3d DimensionConverter::DepthTo3D(int x, int y, short depthValue);
	static cv::Point2i DimensionConverter::From3DPointToRGBCameraFrame(cv::Point3d depthPoint);
};

#endif