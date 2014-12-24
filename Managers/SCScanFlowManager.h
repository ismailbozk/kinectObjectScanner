#ifndef SCSCANFLOWMANAGER_H
#define SCSCANFLOWMANAGER_H

#include "..\Models\SCPoint3D.h"

#include <opencv2/core/core.hpp>

class SCScanFlowManager
{
public:
	static SCScanFlowManager *SharedScannerManager();

	cv::Mat lastTransformationMat;
	std::vector<SCPoint3D> wholePointCloud;

protected:
	SCScanFlowManager(void);
	~SCScanFlowManager(void);

private:
	
};

#endif