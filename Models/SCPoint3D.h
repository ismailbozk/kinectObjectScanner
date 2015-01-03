#ifndef SCPOINT3D_H
#define SCPOINT3D_H

#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/core.hpp>

class SCPoint3D
{
public:
	SCPoint3D(void);
	SCPoint3D(cv::Point3d point, cv::Vec3b color);
	~SCPoint3D(void);

	cv::Point3d pt;
	cv::Vec3b color;	//bgr order [0],[1],[2]
};

#endif