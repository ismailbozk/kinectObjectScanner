#include "PlayGround.h"

#include <iostream>

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
#include "Models\SCPoint3D.h"


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

	vector<Match3D> *matches3D = TransformationUtility::Create3DMatchPoints(mask, matches, kinectModelTrain, keyPointsTrain, kinectModelTest, keyPointsTest);
	
	///test
	/*vector<Match3D> *matches3DTest = new vector<Match3D>();
	
	Point3f trA = Point3f(0,-1,1);
	Point3f teA = Point3f(1,0,1);
	(*matches3DTest).push_back(Match3D(teA, trA));
	Point3f trB = Point3f(-1,-1,1);
	Point3f teB = Point3f(1,-1,1);
	(*matches3DTest).push_back(Match3D(teB, trB));
	Point3f trC = Point3f(-1,0,1);
	Point3f teC = Point3f(0,-1,1);
	(*matches3DTest).push_back(Match3D(teC, trC));
	Point3f trD = Point3f(-1,1,1);
	Point3f teD = Point3f(-1,-1,1);
	(*matches3DTest).push_back(Match3D(teD, trD));
	vector<Match3D> *inlierMatches = TransformationUtility::RANSAC(*matches3DTest, 3, 0.2);
	cv::Matx44f *transformationMatrixTest = TransformationUtility::CreateTransformation(*inlierMatches);*/

	//////////////////////////////////////

	vector<Match3D> *inlierMatches3D = TransformationUtility::RANSAC(*matches3D, 100000, 0.25f);

	cv::Matx44f *transformationMatrix = TransformationUtility::CreateTransformation(*inlierMatches3D);
	vector<SCPoint3D> *trainPC = TransformationUtility::Transform(kinectModelTrain, cv::Matx44f::eye());
	vector<SCPoint3D> *testPC = TransformationUtility::Transform(kinectModelTest, (*transformationMatrix));

	vector<SCPoint3D> *allPC = new vector<SCPoint3D>();
	(*allPC).reserve((*trainPC).size() + (*testPC).size());

	(*allPC).insert( (*allPC).end(), (*trainPC).begin(), (*trainPC).end());
	(*allPC).insert( (*allPC).end(), (*testPC).begin(), (*testPC).end());

	DrawUtility::WritePLYFile("C:\\test.ply", *allPC);
}