#include <opencv2/core/core.hpp>
#include <vector>

#ifndef BASEKINECTMODEL_H
#define BASEKINECTMODEL_H

class BaseKinectModel
{
public:
	BaseKinectModel();
	BaseKinectModel(cv::Mat, std::vector<short>, std::vector<cv::Point3d>);
	BaseKinectModel(cv::Mat, cv::Mat, std::vector<short>, std::vector<cv::Point3d>);


	cv::Mat Image;
	cv::Mat grayImage;						
	std::vector<short> depth;					//depthData
	std::vector<cv::Point3d> pointCloud;		//pointcloud in 3D
};

#endif
