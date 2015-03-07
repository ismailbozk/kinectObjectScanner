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

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>

using namespace pcl;
using namespace cv;
using namespace std;

static short surfHessianThreshold = 300;
static float uniquenessThreshold = 0.8f;
static short k = 2;
static float ransacThreshold = 0.2f;

PlayGround::PlayGround(void)
{}

void PlayGround::startToPlay()
{
	//-- Step 0: load test data
	std::string filePrefix1 = "father4", filePrefix2 = "father5";

	BaseKinectModel kinectModelTrain = SerializationUtility::getKinectDataWithFilePrefix(filePrefix1);
	BaseKinectModel kinectModelTest = SerializationUtility::getKinectDataWithFilePrefix(filePrefix2);
	cout << "Test data loaded." << endl;

	//-- Step 1: Detect the keypoints using SURF Detector
	SurfFeatureDetector surfDetector (surfHessianThreshold);

	std::vector<cv::KeyPoint> keyPointsTrain, keyPointsTest;

	surfDetector.detect(kinectModelTrain.grayImage, keyPointsTrain);
	surfDetector.detect(kinectModelTest.grayImage, keyPointsTest);
	cout << "Step 1: rgb frame features extracted." << endl;

	//-- Step 2: Calculate descriptors (feature vectors)
	SurfDescriptorExtractor extractor;
	Mat descriptorsTrain, descriptorsTest;
	
	extractor.compute(kinectModelTrain.grayImage, keyPointsTrain, descriptorsTrain);
	extractor.compute(kinectModelTest.grayImage, keyPointsTest, descriptorsTest);
	cout << "Step 2: Feature descriptors extracted" << endl;

	//-- Step 3: Matching descriptor vectors with a brute force matcher
	BFMatcher matcher(NORM_L2, false);

	vector<vector<DMatch>> matches;
	Mat maskKnn;
	//matcher.match(descriptorsTrain, descriptorsTest, matches);
	matcher.knnMatch(descriptorsTest, descriptorsTrain, matches, k, maskKnn, false);

	vector<bool> mask(matches.size(), true);
	std::vector<DepthScale> depthScales = Features2DUtility::CreateInlierDepthScales(kinectModelTrain, keyPointsTrain);
	
	Features2DUtility::VoteForUniqueness(matches, 0.8f, mask);
	cout << "Step 3: Consecutive rgb frame features were matched." << endl;

	//DrawUtility::DrawMatches(mask, matches, kinectModelTest.grayImage, keyPointsTest, kinectModelTrain.grayImage, keyPointsTrain);

	///-- test for transform utility
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

	//-- Step 4: transformation
	vector<Match3D> *matches3D = TransformationUtility::Create3DMatchPoints(mask, matches, kinectModelTrain, keyPointsTrain, kinectModelTest, keyPointsTest);
	vector<Match3D> *inlierMatches3D = TransformationUtility::RANSAC(*matches3D, 10000, ransacThreshold);

	cv::Matx44f *transformationMatrix = TransformationUtility::CreateTransformation(*inlierMatches3D);
	vector<SCPoint3D> *trainPC = TransformationUtility::Transform(kinectModelTrain, cv::Matx44f::eye());
	vector<SCPoint3D> *testPC = TransformationUtility::Transform(kinectModelTest, (*transformationMatrix));

	vector<SCPoint3D> *allPC = new vector<SCPoint3D>();
	(*allPC).reserve((*trainPC).size() + (*testPC).size());

	(*allPC).insert( (*allPC).end(), (*trainPC).begin(), (*trainPC).end());
	(*allPC).insert( (*allPC).end(), (*testPC).begin(), (*testPC).end());
	cout << "Step 4: Consecutive frame 3D point clouds were prepared." << endl;

	//DrawUtility::WritePLYFile("C:\\test.ply", *allPC);
	
	//-- Step 5: Surface Reconstruction
	PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>());
	for (int i = 0; i < (*allPC).size(); i++)
	{
		PointXYZ pt;
		pt.x = (*allPC)[i].pt(0,0);
		pt.y = (*allPC)[i].pt(0,0);
		pt.z = (*allPC)[i].pt(0,0);
		//pt.r = (*allPC)[i].color[2];
		//pt.g = (*allPC)[i].color[1];
		//pt.b = (*allPC)[i].color[0];
		(*cloud).push_back(pt);
	}

	MovingLeastSquares<PointXYZ, PointXYZ> mls;
	mls.setInputCloud (cloud);
	mls.setSearchRadius (0.01);
	mls.setPolynomialFit (true);
	mls.setPolynomialOrder (2);
	mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
	mls.setUpsamplingRadius (0.005);
	mls.setUpsamplingStepSize (0.003);
	PointCloud<PointXYZ>::Ptr cloud_smoothed (new PointCloud<PointXYZ> ());
	mls.process (*cloud_smoothed);
	NormalEstimationOMP<PointXYZ, Normal> ne;
	ne.setNumberOfThreads (8);
	ne.setInputCloud (cloud_smoothed);
	ne.setRadiusSearch (0.01);
	Eigen::Vector4f centroid;
	compute3DCentroid (*cloud_smoothed, centroid);
	ne.setViewPoint (centroid[0], centroid[1], centroid[2]);
	PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal> ());
	ne.compute (*cloud_normals);
	for (size_t i = 0; i < cloud_normals->size (); ++i)
	{
		cloud_normals->points[i].normal_x *= -1;
		cloud_normals->points[i].normal_y *= -1;
		cloud_normals->points[i].normal_z *= -1;
	}
	PointCloud<PointNormal>::Ptr cloud_smoothed_normals (new PointCloud<PointNormal> ());
	concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);

	Poisson<PointNormal> poisson;
	poisson.setDepth (9);
	poisson.setInputCloud
	(cloud_smoothed_normals);
	PolygonMesh mesh;
	poisson.reconstruct (mesh);
	io::saveVTKFile ("C:\\testMesh", mesh);
}