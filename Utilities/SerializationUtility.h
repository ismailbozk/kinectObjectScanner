#include <opencv2/core/core.hpp>

#ifndef SERIALIZATIONUTILITY_H
#define SERIALIZATIONUTILITY_H

class SerializationUtility
{
public:
	static cv::Mat getGrayScaleImage(std::string);
	static std::vector<short> getDepthData(std::string filePrefix);
	static std::vector<cv::Vec3b> getColorData(std::string filePrefix);
};

#endif