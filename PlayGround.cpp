#include "PlayGround.h"

#include <opencv2/core/core.hpp>

#include "Models\BaseKinectModel.h"
#include "Utilities\SerializationUtility.h"


PlayGround::PlayGround(void)
{}

void PlayGround::startToPlay()
{
	cv::Mat image = SerializationUtility::getGrayScaleImage("father1");
	std::vector<short> depth = SerializationUtility::getDepthData("father1");

	BaseKinectModel testModel = BaseKinectModel::BaseKinectModelWithGrayImage(image, depth);
}