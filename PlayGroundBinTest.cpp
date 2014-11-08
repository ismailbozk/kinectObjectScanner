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

static short surfHessianThreshold = 10000;
static float uniquenessThreshold = 0.8f;
static short k = 2;
static float uniqThreshold = 0.8f;

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

#pragma region Ordinary Way
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

	vector<bool> mask(matches.size(), true);
	Features2DUtility::VoteForUniqueness(matches, uniqThreshold, mask);
	auto endMatch = std::chrono::high_resolution_clock::now();
	std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(endMatch-beginMatch).count() << "ms " << "Knn Match" <<  std::endl;
	//==========================================================
#pragma endregion

	vector<int> bins;//be warned put the numbers in to a increasing order
	bins.push_back(1);
	bins.push_back(10000);
	bins.push_back(20000);
	bins.push_back(30000);
	bins.push_back(40000);
	bins.push_back(50000);
	bins.push_back(60000);
	bins.push_back(80000);
	bins.push_back(100000);
	bins.push_back(130000);
	bins.push_back(200000);
	bins.push_back(1000000000);

#pragma region Bin Test
	//Get depthScales
	vector<DepthScale> depthScales1 = Features2DUtility::GetInlierDepthScales(f1, keyPoints1);
	vector<DepthScale> depthScales2 = Features2DUtility::GetInlierDepthScales(f2, keyPoints2);

	//Divide all DS, keypoints and descriptors into Bins by their global indices
	vector<vector<int>> kpIndicesDividedBins1 = PlayGroundBinTest::GetDsIndicesDivideByBins(depthScales1, bins);
	vector<vector<int>> kpIndicesDividedBins2 = PlayGroundBinTest::GetDsIndicesDivideByBins(depthScales2, bins);

	vector<vector<bool>> maskBT(kpIndicesDividedBins1.size());//for comparing result I need to keep track of the overall mask.

	for(int i = 0; i < kpIndicesDividedBins1.size(); i++)
	{
		vector<int> kpIndices1 = kpIndicesDividedBins1[i];
		vector<int> kpIndices2 = kpIndicesDividedBins2[i];

		//-- Step 1: Gather the keypoints using SURF Detector
		std::vector<cv::KeyPoint> keyPointsBT1;
		std::vector<cv::KeyPoint> keyPointsBT2;

		for (int i = 0; i < kpIndices1.size(); i++)
		{
			keyPointsBT1.push_back(keyPoints1[kpIndices1[i]]);
		}
		for (int i = 0; i < kpIndices2.size(); i++)
		{
			keyPointsBT2.push_back(keyPoints2[kpIndices2[i]]);
		}
		//==========================================================		

		//-- Step 2: Calculate descriptors (feature vectors)
		SurfDescriptorExtractor extractor;
		Mat descriptorsBT1, descriptorsBT2;

		extractor.compute(f1.grayImage, keyPointsBT1, descriptorsBT1);
		extractor.compute(f2.grayImage, keyPointsBT2, descriptorsBT2);
		//==========================================================

		//-- Step 3: Matching descriptor vectors with a brute force matcher
		BFMatcher matcher(NORM_L2, false);

		std::vector<vector<DMatch>> matchesBT;
		Mat maskKnnBT;
		matcher.knnMatch(descriptorsBT1, descriptorsBT2, matchesBT, k, maskKnnBT, false);

		vector<bool> maskBTTemp(matchesBT.size(), true);
		Features2DUtility::VoteForUniqueness(matchesBT, uniqThreshold, maskBTTemp);
		maskBT[i] = maskBTTemp;
		//==========================================================									
	}



#pragma endregion
}

std::vector<std::vector<int>> PlayGroundBinTest::GetDsIndicesDivideByBins(std::vector<DepthScale> depthScales, std::vector<int> bins)
{
	std::vector<std::vector<int>> dsIndices(bins.size());
	for (int dsI = 0; dsI < depthScales.size(); dsI++)
	{
		int ds = depthScales[dsI].depthScale;
		for (int binI = 0; binI < bins.size() - 1; binI++)
		{
			int lowerBound = bins[binI];
			int upperBound = bins[binI+1];

			if (ds > lowerBound && ds <= upperBound)
			{
				dsIndices[binI].push_back(dsI);
				break;
			}
		}
	}
	return dsIndices;
}
