#include "SCScanFlowManager.h"

static SCScanFlowManager *sharedScannerManagerInstance;

SCScanFlowManager *SCScanFlowManager::SharedScannerManager()
{
	if(!sharedScannerManagerInstance)
	{
		sharedScannerManagerInstance = new SCScanFlowManager();
	}
	return sharedScannerManagerInstance;
}

SCScanFlowManager::SCScanFlowManager(void)
{
	this->lastTransformationMat = cv::Mat::eye(4, 4, CV_64F);		//default value of last transformationmatrix is identity matrix
	this->wholePointCloud = std::vector<SCPoint3D>();
}

SCScanFlowManager::~SCScanFlowManager(void)
{
	
}
