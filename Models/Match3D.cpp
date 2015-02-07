#include "Match3D.h"


Match3D::Match3D(void)
{
}

Match3D::Match3D(cv::Point3f queryPair, cv::Point3f trainPair)
{
	Match3D::queryPair = queryPair;
	Match3D::trainPair = trainPair;
}

Match3D::~Match3D(void)
{

}
