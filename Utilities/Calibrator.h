#ifndef CALIBRATOR_H
#define CALIBRATOR_H

typedef enum
{
	CalibratorDeviceKinectV1
}CalibratorDevice;

class Calibrator
{
public:
	static cv::Point3d CalibrateDepthFrameOnDepthCloud(cv::Point3d point)
;

};

#endif