#include "BaseKinectModel.h"

#include "..\Utilities\DimensionConverter.h"

BaseKinectModel::BaseKinectModel()
{}


BaseKinectModel BaseKinectModel::BaseKinectModelWithGrayImage(cv::Mat grayImage, std::vector<short> rawDepthData)
{
	BaseKinectModel model = BaseKinectModel();
	model.grayImage = grayImage;

	int sizeOfTheFrame = grayImage.rows * grayImage.cols;

	model.calibratedDepth = std::vector<short>(sizeOfTheFrame);
	model.pointCloud = std::vector<cv::Point3d>(sizeOfTheFrame);

	for (int i = 0; i < rawDepthData.size() ; i++)
	{
		if (rawDepthData[i] > 1)
		{//if raw depth data is valid. the very first generetion kinect currently can detect distance between 1000 mm to 3500 mm 
			int x = i % model.grayImage.cols;
			int y = i / model.grayImage.cols;
			cv::Point3d point3D = DimensionConverter::DepthTo3D(x, y, rawDepthData[i]);

			cv::Point2i calibratedXYPoint = DimensionConverter::From3DPointToRGBCameraFrame(point3D);		//coordinates on gray image

			if (calibratedXYPoint.x >= 0 && calibratedXYPoint.x < model.grayImage.cols &&
				calibratedXYPoint.y >= 0 && calibratedXYPoint.y < model.grayImage.rows)
			{//if calibratedImage point is intersects with the original image dimensions
				int calibratedIndex = (calibratedXYPoint.y * model.grayImage.cols) + calibratedXYPoint.x;

				model.calibratedDepth[calibratedIndex] = rawDepthData[i];
				model.pointCloud[calibratedIndex] = point3D;
			}
		}
	}

	return model;
}


