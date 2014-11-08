#include "PlayGroundBinTest.h"

#include <opencv2/core/core.hpp>
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>

#include "Utilities\Features2DUtility.h"
#include "Utilities\SerializationUtility.h"
#include "Utilities\DrawUtility.h"

#include "Models\BaseKinectModel.h"
#include "Models\DepthScale.h"

#include <chrono>

using namespace cv;
using namespace std;

static short surfHessianThreshold = 300;
static float uniquenessThreshold = 0.8f;
static short k = 2;
//the whole purpose of that experiment is dividing the all descriptors into the bins for model and observed descriptor by using thier depthscale values
//then use that divided depthscales match the descriptors in the same bins
//b/c we know that only if theri depthscales are similar then we can say that they are true candidate matches
PlayGroundBinTest::PlayGroundBinTest(void)
{}

void PlayGroundBinTest::startToPlay()
{
	//Read test Data
	auto beginRead = std::chrono::high_resolution_clock::now();
	std::string filePrefix1 = "father1", filePrefix2 = "father2";

	BaseKinectModel f1 = SerializationUtility::getKinectDataWithFilePrefix(filePrefix1);
	BaseKinectModel f2 = SerializationUtility::getKinectDataWithFilePrefix(filePrefix2);
	auto endRead = std::chrono::high_resolution_clock::now();
	std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(endRead-beginRead).count() << "ms " << "Read Test Data" << std::endl;
	//=========================================================


	//-- Step 1: Detect the keypoints using SURF Detector
	auto beginSurf = std::chrono::high_resolution_clock::now();
	SurfFeatureDetector surfDetector (surfHessianThreshold);
	std::vector<cv::KeyPoint> keyPoints1, keyPoints2;

	surfDetector.detect(f1.grayImage, keyPoints1);
	surfDetector.detect(f2.grayImage, keyPoints2);
	auto endSurf = std::chrono::high_resolution_clock::now();
	std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(endSurf-beginSurf).count() << "ms " << "Surf Detect" << std::endl;
	//==========================================================

	//-- Step 2: Calculate descriptors (feature vectors)
	auto beginSurfExtract = std::chrono::high_resolution_clock::now();
	SurfDescriptorExtractor extractor;
	Mat descriptors1, descriptors2;
	
	extractor.compute(f1.grayImage, keyPoints1, descriptors1);
	extractor.compute(f2.grayImage, keyPoints2, descriptors2);
	auto endSurfExtract = std::chrono::high_resolution_clock::now();
	std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(endSurfExtract-beginSurfExtract).count() << "ms " << "Surf Extract" << std::endl;
	//==========================================================

	//-- Step 3: Matching descriptor vectors with a brute force matcher
	auto beginMatch = std::chrono::high_resolution_clock::now();
	BFMatcher matcher(NORM_L2, false);

	std::vector<vector<DMatch>> matches;
	Mat maskKnn;
	//matcher.match(descriptors1, descriptors2, matches);
	matcher.knnMatch(descriptors1, descriptors2, matches, k, maskKnn, false);

	std::vector<DepthScale> depthScales = Features2DUtility::GetInlierDepthScales(f1, keyPoints1);

	vector<bool> mask(matches.size(), true);
	Features2DUtility::VoteForUniqueness(matches, 0.8f, mask);
	auto endMatch = std::chrono::high_resolution_clock::now();
	std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(endMatch-beginMatch).count() << "ms " << "Knn Match" <<  std::endl;
	//==========================================================

}