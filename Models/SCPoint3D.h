#ifndef SCPOINT3D_H
#define SCPOINT3D_H

#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/core.hpp>

class SCPoint3D
{
public:
	SCPoint3D(void);
	SCPoint3D(cv::Matx14f point, cv::Vec3b color);
	~SCPoint3D(void);

	cv::Matx14f pt;		//pt(0,0) = x; pt(0,1) = y; pt(0,2) = z; pt(0,3) = 1.0f; 
	cv::Vec3b color;	//bgr order [0],[1],[2]
};

#endif