#ifndef MATCH3D_H
#define MATCH3D_H

#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/core.hpp>

class Match3D
{
public:
	Match3D(void);
	~Match3D(void);

	Match3D::Match3D(cv::Point3f queryPair, cv::Point3f trainPair);

	cv::Point3f queryPair;
	cv::Point3f trainPair;
};

#endif

