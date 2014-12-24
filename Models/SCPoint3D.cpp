#include "SCPoint3D.h"

SCPoint3D::SCPoint3D(void)
{}

SCPoint3D::SCPoint3D(cv::Point3d point, cv::Vec3b color)
{
	this->pt = point;
	this->color = color;
}

SCPoint3D::~SCPoint3D(void)
{}
