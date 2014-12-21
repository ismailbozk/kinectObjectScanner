#include "PlayGround.h"

#include <opencv2/core/core.hpp>
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>

#include "Utilities\Features2DUtility.h"
#include "Utilities\SerializationUtility.h"
#include "Utilities\DrawUtility.h"
#include "Utilities\TransformationUtility.h"

#include "Models\BaseKinectModel.h"
#include "Models\DepthScale.h"
#include "Models\Match3D.h"



using namespace cv;
using namespace std;

static short surfHessianThreshold = 300;
static float uniquenessThreshold = 0.8f;
static short k = 2;

PlayGround::PlayGround(void)
{}

void PlayGround::startToPlay()
{
	std::string filePrefix1 = "father2", filePrefix2 = "father1";

	BaseKinectModel kinectModelTrain = SerializationUtility::getKinectDataWithFilePrefix(filePrefix1);
	BaseKinectModel kinectModelTest = SerializationUtility::getKinectDataWithFilePrefix(filePrefix2);

	//-- Step 1: Detect the keypoints using SURF Detector
	SurfFeatureDetector surfDetector (surfHessianThreshold);

	std::vector<cv::KeyPoint> keyPointsTrain, keyPointsTest;

	surfDetector.detect(kinectModelTrain.grayImage, keyPointsTrain);
	surfDetector.detect(kinectModelTest.grayImage, keyPointsTest);

	//-- Step 2: Calculate descriptors (feature vectors)

	SurfDescriptorExtractor extractor;
	Mat descriptorsTrain, descriptorsTest;
	
	extractor.compute(kinectModelTrain.grayImage, keyPointsTrain, descriptorsTrain);
	extractor.compute(kinectModelTest.grayImage, keyPointsTest, descriptorsTest);

	//-- Step 3: Matching descriptor vectors with a brute force matcher
	BFMatcher matcher(NORM_L2, false);

	vector<vector<DMatch>> matches;
	Mat maskKnn;
	//matcher.match(descriptorsTrain, descriptorsTest, matches);
	matcher.knnMatch(descriptorsTest, descriptorsTrain, matches, k, maskKnn, false);

	vector<bool> mask(matches.size(), true);
	std::vector<DepthScale> depthScales = Features2DUtility::CreateInlierDepthScales(kinectModelTrain, keyPointsTrain);
	
	Features2DUtility::VoteForUniqueness(matches, 0.8f, mask);

	DrawUtility::DrawMatches(mask, matches, kinectModelTest.grayImage, keyPointsTest, kinectModelTrain.grayImage, keyPointsTrain);

	vector<Match3D> matches3D = TransformationUtility::Create3DMatchPoints(mask, matches, kinectModelTrain, keyPointsTrain, kinectModelTest, keyPointsTest);
	cv::Mat4d transformationMatrix = TransformationUtility::CreateTransformation(matches3D);


}