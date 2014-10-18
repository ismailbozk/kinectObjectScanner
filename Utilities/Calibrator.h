#include <opencv2/core/core.hpp>

#ifndef CALIBRATOR_H
#define CALIBRATOR_H

typedef enum
{
	CalibratorDeviceKinectV1
}CalibratorDevice;

class Calibrator
{
public:
	static cv::Point3d Calibrator::CalibrateDepthFrameOnDepthCloud(cv::Point3d point);

};

#endif