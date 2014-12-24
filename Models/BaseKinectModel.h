#ifndef BASEKINECTMODEL_H
#define BASEKINECTMODEL_H

#include <opencv2/core/core.hpp>
#include <vector>

class BaseKinectModel
{
public:
	//default constructors
	BaseKinectModel();
	~BaseKinectModel(void);

	//essential Constructor to obtain depthdata and cloud data from raw depth data
	static BaseKinectModel BaseKinectModel::BaseKinectModelWithGrayImage(cv::Mat, std::vector<short>);
	//static BaseKinectModel BaseKinectModel::BaseKinectModelWithColoredImage(cv::Mat, std::vector<short>);		//use this method when colored images were captured via Kinect camera

	cv::Mat image;								//color image					
	cv::Mat grayImage;							//grayScale image
	std::vector<short> calibratedDepth;			//depthData
	std::vector<cv::Point3d> pointCloud;		//local pointcloud in 3D
};

#endif
