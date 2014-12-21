#include "Calibrator.h"

#include <opencv2/core/core.hpp>

#pragma message( WARNING( "try using double instead of float here" ) )
double R11 = 9.9984628826577793e-01f; double R12 = 1.2635359098409581e-03f; double R13 = -1.7487233004436643e-02f;
double R21 = -1.4779096108364480e-03f; double R22 = 9.9992385683542895e-01f; double R23 = -1.2251380107679535e-02f;
double R31 = 1.7470421412464927e-02f; double R32 = 1.2275341476520762e-02f; double R33 = 9.9977202419716948e-01f;

double T11 = 1.9985242312092553e-02f; double T12 = -7.4423738761617583e-04f; double T13 = -1.0916736334336222e-02f;


cv::Point3d Calibrator::CalibrateDepthFrameOnDepthCloud(cv::Point3d point)
{
	cv::Point3d *result  = new cv::Point3d();

	result->x = (point.x * R11) + (point.y * R21) + (point.z * R31);
	result->y = (point.x * R12) + (point.y * R22) + (point.z * R32);
	result->z = (point.x * R13) + (point.y * R23) + (point.z * R33);

	result->x += T11;
	result->y += T12;
	result->z += T13;

	return *result;
}
