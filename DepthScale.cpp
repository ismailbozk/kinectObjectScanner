#include "DepthScale.h"

DepthScale::DepthScale(short depth, short radius)
{
	DepthScale::depth = depth;
	DepthScale::radius = radius;
	DepthScale::depthScale = depth * radius;
}

