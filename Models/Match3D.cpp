#include "Match3D.h"


Match3D::Match3D(void)
{
}

Match3D::Match3D(cv::Point3d queryPair, cv::Point3d trainPair)
{
	Match3D::queryPair = queryPair;
	Match3D::trainPair = trainPair;
}

Match3D::~Match3D(void)
{

}
