#ifndef TRANSFORMATIONUTILITY_H
#define TRANSFORMATIONUTILITY_H

#include <opencv2/nonfree/nonfree.hpp>

#include <vector>

#include "../Models/BaseKinectModel.h"
#include "../Models/Match3D.h"
#include "../Models/SCPoint3D.h"

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

class TransformationUtility
{
public:


	static std::shared_ptr<cv::Matx44f> TransformationUtility::IterativePointCloudMatchTransformation(PointCloud::Ptr trainingPointCloud, PointCloud::Ptr testPointCloud);


	static std::vector<Match3D> *TransformationUtility::Create3DMatchPoints(std::vector<bool> &mask, std::vector<std::vector<cv::DMatch>> &matches, BaseKinectModel &trainKinectModel, std::vector<cv::KeyPoint> &trainKeypoints, BaseKinectModel &testKinectModel, std::vector<cv::KeyPoint> &testKeypoints);
	static cv::Matx44f *TransformationUtility::CreateTransformation(std::vector<Match3D> &matches3D);

	static std::vector<SCPoint3D> *TransformationUtility::Transform(BaseKinectModel &testModel, cv::Matx44f &transformationMatrix);

	static std::vector<Match3D> *TransformationUtility::RANSAC(std::vector<Match3D> &matches3D, int numberOfIteration, float threshold);

	static cv::Matx14f TransformationUtility::TransformSinglePoint2Matrix(cv::Point3f &point, cv::Matx44f &transformationMatrix);
	static cv::Point3f TransformationUtility::TransformSinglePoint2Point(cv::Point3f &point, cv::Matx44f &transformationMatrix);
	static void TransformationUtility::TransformSingleMatrix(cv::Matx14f &pt, cv::Matx44f &transformationMatrix);

private:
	static std::vector<unsigned int> *TransformationUtility::Generate3UniqueRandom(unsigned int ceil);
	static bool TransformationUtility::IsTransformationMatrixRightHanded(cv::Matx44f &transformation);
	static void TransformationUtility::EuclideanDistanceBetweenTwoPoint(double &euclideanDistance, cv::Point3f &pointA, cv::Point3f &pointB);
};

#endif

