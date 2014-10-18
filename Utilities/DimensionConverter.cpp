#include "DimensionConverter.h"

#include "Calibrator.h"
#include <opencv2/core/core.hpp>
#include <vector>

using namespace cv;

const double fx_rgb = 5.2921508098293293e+02;
const double fy_rgb = 5.2556393630057437e+02;
const double cx_rgb = 3.2894272028759258e+02;
const double cy_rgb = 2.6748068171871557e+02;

const double fx_d = 1.0 / 5.9421434211923247e+02;
const double fy_d = 1.0 / 5.9104053696870778e+02;
const double cx_d = 3.3930780975300314e+02;
const double cy_d = 2.4273913761751615e+02;

cv::Point3d DimensionConverter::DepthToWorld(int x, int y, short depthValue)
{
	cv::Point3d result = cv::Point3d();
	
	double depth = (double)(depthValue) / 1000; // mm to meter

	result.x = ((x - cx_d) * depth * fx_d);
    result.y = ((y - cy_d) * depth * fy_d);
    result.z = depth;

	return result;
}

cv::Point2i DimensionConverter::DepthPointCloudToRGBCameraFrame(cv::Point3d depthPoint)
{
	cv::Point3d transformedPoint = Calibrator::CalibrateDepthFrameOnDepthCloud(depthPoint);

	double invZ = 1.0 / transformedPoint.z;

	Point2i result = Point2i();

    double xValue = ((transformedPoint.x * fx_rgb * invZ) + cx_rgb);
    xValue += 3.0f;
    result.x = (int)xValue;

    double yValue = ((transformedPoint.y * fy_rgb * invZ) + cy_rgb);
    yValue -= 14.7f;
	result.y = (int)yValue;

	return result;
}