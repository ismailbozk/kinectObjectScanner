#include "TransformationUtility.h"

#include <opencv2/legacy/legacy.hpp>

#include "../Macros.h"
#include <iostream>

#include <../eigen/Eigen/Dense>
#include <../eigen/Eigen/SVD>


using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;



	//our visualizer
	pcl::visualization::PCLVisualizer *p;
	//its left and right viewports
	int vp_1, vp_2;

using namespace std;
using namespace cv;
using namespace Eigen;

#pragma region PointCloud iterative registration

//convenient structure to handle our pointclouds
struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud (new PointCloud) {};
};

struct PCDComparator
{
  bool operator () (const PCD& p1, const PCD& p2)
  {
    return (p1.f_name < p2.f_name);
  }
};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const PointNormalT &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the first viewport of the visualizer
 *
 */
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
  p->removePointCloud ("vp1_target");
  p->removePointCloud ("vp1_source");

  PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
  p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
  p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

  PCL_INFO ("Press q to begin the registration.\n");
  p-> spin();
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
 *
 */
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
  p->removePointCloud ("source");
  p->removePointCloud ("target");


  PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
  if (!tgt_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");

  PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
  if (!src_color_handler.isCapable ())
      PCL_WARN ("Cannot create curvature color handler!");


  p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
  p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

  p->spinOnce();
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of PCD files that we want to register together
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param models the resultant vector of point cloud datasets
  */
void loadData (int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD> > &models)
{
  
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
  * \param cloud_src the source PointCloud
  * \param cloud_tgt the target PointCloud
  * \param output the resultant aligned source PointCloud
  * \param final_transform the resultant transform between source and target
  */
void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output, Eigen::Matrix4f &final_transform, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src (new PointCloud);
  PointCloud::Ptr tgt (new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }


  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  norm_est.setSearchMethod (tree);
  norm_est.setKSearch (30);
  
  norm_est.setInputCloud (src);
  norm_est.compute (*points_with_normals_src);
  pcl::copyPointCloud (*src, *points_with_normals_src);

  norm_est.setInputCloud (tgt);
  norm_est.compute (*points_with_normals_tgt);
  pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues (alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon (1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance (0.1);  
  // Set the point representation
  reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

  reg.setInputSource (points_with_normals_src);
  reg.setInputTarget (points_with_normals_tgt);



  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations (2);
  for (int i = 0; i < 30; ++i)
  {
    PCL_INFO ("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource (points_with_normals_src);
    reg.align (*reg_result);

		//accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
      reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
    
    prev = reg.getLastIncrementalTransformation ();

    // visualize current state
    showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }

	//
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

  p->removePointCloud ("source");
  p->removePointCloud ("target");

  PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
  p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
  p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

	PCL_INFO ("Press q to continue the registration.\n");
  p->spin ();

  p->removePointCloud ("source"); 
  p->removePointCloud ("target");

  //add the source to the transformed target
  *output += *cloud_src;
  
  final_transform = targetToSource;
 }

std::shared_ptr<cv::Matx44f> TransformationUtility::IterativePointCloudMatchTransformation(PointCloud::Ptr trainingPointCloud, PointCloud::Ptr testPointCloud)
{
	// Create a PCLVisualizer object
	p = new pcl::visualization::PCLVisualizer ("Pairwise Incremental Registration example");
	p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
	p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

	PointCloud::Ptr result (new PointCloud), source, target;
	Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), pairTransform;

	source = trainingPointCloud;
	target = testPointCloud;

    // Add visualization data
    showCloudsLeft(source, target);

    PointCloud::Ptr temp (new PointCloud);
    pairAlign (source, target, temp, pairTransform, true);

    //transform current pair into the global transform
    pcl::transformPointCloud (*temp, *result, GlobalTransform);

    //update the global transform
    GlobalTransform = GlobalTransform * pairTransform;

		//save aligned pair, transformed into the first cloud's frame
    std::stringstream ss;
    pcl::io::savePCDFile (ss.str (), *result, true);

	std::shared_ptr<cv::Matx44f> iterativeTransformationMatrix(new cv::Matx44f());
	
	(*iterativeTransformationMatrix)(0,0) = pairTransform(0,0);(*iterativeTransformationMatrix)(0,1) = pairTransform(1,0);
	(*iterativeTransformationMatrix)(0,2) = pairTransform(2,0);(*iterativeTransformationMatrix)(0,3) = pairTransform(3,0);

	(*iterativeTransformationMatrix)(1,0) = pairTransform(0,1);(*iterativeTransformationMatrix)(1,1) = pairTransform(1,1);
	(*iterativeTransformationMatrix)(1,2) = pairTransform(2,1);(*iterativeTransformationMatrix)(1,3) = pairTransform(3,1);

	(*iterativeTransformationMatrix)(2,0) = pairTransform(0,2);(*iterativeTransformationMatrix)(2,1) = pairTransform(1,2);
	(*iterativeTransformationMatrix)(2,2) = pairTransform(2,2);(*iterativeTransformationMatrix)(2,3) = pairTransform(3,2);

	(*iterativeTransformationMatrix)(3,0) = pairTransform(0,3);(*iterativeTransformationMatrix)(3,1) = pairTransform(1,3);
	(*iterativeTransformationMatrix)(3,2) = pairTransform(2,3);(*iterativeTransformationMatrix)(3,3) = pairTransform(3,3);

	return iterativeTransformationMatrix;
}


#pragma endregion


std::vector<Match3D> *TransformationUtility::Create3DMatchPoints(std::vector<bool> &mask, std::vector<std::vector<cv::DMatch>> &matches, BaseKinectModel &trainKinectModel, std::vector<cv::KeyPoint> &trainKeypoints, BaseKinectModel &testKinectModel, std::vector<cv::KeyPoint> &testKeypoints)
{
	vector<Match3D> *matches3D = new vector<Match3D>();
	for (int i = 0; i < mask.size(); i++)
	{
		if (mask[i] == true)
		{
			DMatch currentMatch = matches[i][0];
			KeyPoint testPairKeyPoint = testKeypoints[currentMatch.queryIdx];
			KeyPoint trainPairKeyPoint = trainKeypoints[currentMatch.trainIdx];

			int trainPairIndex = (int)(((trainKinectModel.grayImage.cols) * round(trainPairKeyPoint.pt.y)) + (int)trainPairKeyPoint.pt.x);
			int testPairIndex = (int)(((testKinectModel.grayImage.cols) * round(testPairKeyPoint.pt.y)) + (int)testPairKeyPoint.pt.x);

			if ((trainKinectModel.pointCloud[trainPairIndex].x == 0.0 && trainKinectModel.pointCloud[trainPairIndex].y == 0.0 && trainKinectModel.pointCloud[trainPairIndex].z == 0.0)
				|| (testKinectModel.pointCloud[testPairIndex].x == 0.0 && testKinectModel.pointCloud[testPairIndex].y == 0.0 && testKinectModel.pointCloud[testPairIndex].z == 0.0))
			{///eliminate zero Point3f's
				mask[i] = false;
			}
			else
			{
				Match3D match3d = Match3D(testKinectModel.pointCloud[testPairIndex], trainKinectModel.pointCloud[trainPairIndex]);
				(*matches3D).push_back(match3d);
			}
		}
	}

	return matches3D;
}

cv::Matx44f *TransformationUtility::CreateTransformation(std::vector<Match3D> &matches3D)
{
	cv::Matx44f *transformation = new cv::Matx44f();

#pragma region obtain mid points of matches and seperated consecutive matches
	Point3f trainMidPoint = Point3f(0,0,0);		//d_ mid
	Point3f testMidPoint = Point3f(0,0,0);		//m_ mid
	for (int i = 0; i < matches3D.size(); i++)
	{
		trainMidPoint += matches3D[i].trainPair;
		testMidPoint += matches3D[i].queryPair;
	}
	trainMidPoint.x = trainMidPoint.x / matches3D.size();
	trainMidPoint.y = trainMidPoint.y / matches3D.size();
	trainMidPoint.z = trainMidPoint.z / matches3D.size();
	testMidPoint.x = testMidPoint.x / matches3D.size();
	testMidPoint.y = testMidPoint.y / matches3D.size();
	testMidPoint.z = testMidPoint.z / matches3D.size();
	#pragma endregion

#pragma region pull the all points to around origin midpoints traslated to the 0,0,0 point and finding the H matrix
	double HMatrix11 = 0.0;
    double HMatrix12 = 0.0;
    double HMatrix13 = 0.0;
    double HMatrix21 = 0.0;
    double HMatrix22 = 0.0;
    double HMatrix23 = 0.0;
    double HMatrix31 = 0.0;
    double HMatrix32 = 0.0;
    double HMatrix33 = 0.0;	

	vector<Match3D> matches3DWhichAreTranslatedAroundTheMidPoints = matches3D;

	for (int i = 0; i <matches3DWhichAreTranslatedAroundTheMidPoints.size(); i++)
	{
		matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair -= trainMidPoint;		//d_i - d_ = d_c_i
		matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair -= testMidPoint;		//m_i - m_ = m_c_i

		HMatrix11 += matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair.x * matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair.x;
		HMatrix12 += matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair.x * matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair.y;
		HMatrix13 += matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair.x * matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair.z;

		HMatrix21 += matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair.y * matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair.x;
		HMatrix22 += matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair.y * matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair.y;
		HMatrix23 += matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair.y * matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair.z;

		HMatrix31 += matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair.z * matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair.x;
		HMatrix32 += matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair.z * matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair.y;
		HMatrix33 += matches3DWhichAreTranslatedAroundTheMidPoints[i].trainPair.z * matches3DWhichAreTranslatedAroundTheMidPoints[i].queryPair.z;
	}

#pragma endregion

#pragma region SVD
	/*cv::Matx33f src = cv::Matx33f();

	src(0,0) = HMatrix11; src(0,1) = HMatrix12; src(0,2) = HMatrix13;
	src(1,0) = HMatrix21; src(1,1) = HMatrix22; src(1,2) = HMatrix23;
	src(2,0) = HMatrix31; src(2,1) = HMatrix32; src(2,2) = HMatrix33;

	cv::Matx31f w;
	cv::Matx33f u;
	cv::Matx33f vt;
	cv::SVD::compute(src, w, u, vt, 0);*/


	Matrix3d eigenSrc = Eigen::Matrix3d::Identity();
	eigenSrc <<	HMatrix11, HMatrix21, HMatrix31,
				HMatrix12, HMatrix22, HMatrix32,
				HMatrix13, HMatrix23, HMatrix33;

	Eigen::JacobiSVD<MatrixXd> svd(eigenSrc, ComputeFullU | ComputeFullV);
	Eigen::Matrix3d eigenV = svd.matrixV();
	Eigen::Matrix3d eigenU = svd.matrixU();
#pragma endregion

#pragma region Rotation Matrix
	Eigen::Matrix3d eigenUMultipleVT = eigenU * eigenV.transpose();

	(*transformation)(0,0) = eigenUMultipleVT(0,0);
	(*transformation)(0,1) = eigenUMultipleVT(0,1);
	(*transformation)(0,2) = eigenUMultipleVT(0,2);
	(*transformation)(0,3) = 0.0;

	(*transformation)(1,0) = eigenUMultipleVT(1,0);
	(*transformation)(1,1) = eigenUMultipleVT(1,1);
	(*transformation)(1,2) = eigenUMultipleVT(1,2);
	(*transformation)(1,3) = 0.0;

	(*transformation)(2,0) = eigenUMultipleVT(2,0);
	(*transformation)(2,1) = eigenUMultipleVT(2,1);
	(*transformation)(2,2) = eigenUMultipleVT(2,2);
	(*transformation)(2,3) = 0.0;

	(*transformation)(3,0) = 0.0;
	(*transformation)(3,1) = 0.0;
	(*transformation)(3,2) = 0.0;
	(*transformation)(3,3) = 1.0;

#pragma endregion

#pragma region camera translation
	cv::Matx14f translation;

	cv::Matx14f midTestPointMat = cv::Matx14f();
	midTestPointMat(0,0) = testMidPoint.x;	midTestPointMat(0,1) = testMidPoint.y;
	midTestPointMat(0,2) = testMidPoint.z;	midTestPointMat(0,3) = 1.0;
	
	cv::Matx14f midTrainPointMat = cv::Matx14f();
	midTrainPointMat(0,0) = trainMidPoint.x;	midTrainPointMat(0,1) = trainMidPoint.y;
	midTrainPointMat(0,2) = trainMidPoint.z;	midTrainPointMat(0,3) = 1.0;

	TransformationUtility::TransformSingleMatrix(midTestPointMat, (*transformation));
    translation = midTrainPointMat - midTestPointMat;

	(*transformation)(3,0) = translation(0,0);
	(*transformation)(3,1) = translation(0,1);
	(*transformation)(3,2) = translation(0,2);
	(*transformation)(3,3) = 1.0;
#pragma endregion

	return transformation;
}

std::vector<SCPoint3D> *TransformationUtility::Transform(BaseKinectModel &testModel, cv::Matx44f &transformationMatrix)
{
	std::vector<SCPoint3D> *result = new std::vector<SCPoint3D>();
	(*result).reserve(testModel.pointCloud.size());

	cv::Mat grayImageWith3Channels;
	if (testModel.image.cols == 0)
	{//if no image found then use grayImage
		cvtColor(testModel.grayImage, grayImageWith3Channels, CV_GRAY2RGB);
	}

	for (int i = 0; i < testModel.pointCloud.size(); i++)
	{
		if (testModel.pointCloud[i].z != 0.0 && testModel.pointCloud[i].y != 0.0 && testModel.pointCloud[i].x != 0.0)
		{//if point is valid
			Point3f pt = testModel.pointCloud[i];
			Matx14f ptMatrix = TransformationUtility::TransformSinglePoint2Matrix(pt, transformationMatrix);

			if (testModel.image.cols > 0)
			{//if color image exists
				(*result).push_back(SCPoint3D(ptMatrix,
					testModel.image.at<cv::Vec3b>(i / testModel.image.cols, i % testModel.image.cols)));
			}
			else if (testModel.grayImage.cols > 0)
			{//if gray color image exists
				(*result).push_back(SCPoint3D(ptMatrix,
					grayImageWith3Channels.at<cv::Vec3b>(i / grayImageWith3Channels.cols, i % grayImageWith3Channels.cols)));
			}
			else
			{//if no image found, then use white as color
				cv::Vec3b white(255, 255, 255);
				(*result).push_back(SCPoint3D(ptMatrix,	cv::Vec3b(255, 255, 255)));
			}
		}
	}

	return result;
}

std::vector<Match3D> *TransformationUtility::RANSAC(std::vector<Match3D> &matches3D, int numberOfIteration, float threshold)
{//requeiress min 3 Match3D
	std::vector<unsigned int> bestMatchesIndices;
	int cBest = INT_MIN;

	for (int iteration = 0; iteration < numberOfIteration; iteration++)
	{
		int c = 0;

		std::vector<unsigned int> *random3 = TransformationUtility::Generate3UniqueRandom(matches3D.size());

		std::vector<Match3D> pickedMatches3D = std::vector<Match3D>();
		pickedMatches3D.reserve(3);
		pickedMatches3D.push_back(matches3D[(*random3)[0]]);
		pickedMatches3D.push_back(matches3D[(*random3)[1]]);
		pickedMatches3D.push_back(matches3D[(*random3)[2]]);

		cv::Matx44f *candTransformation = TransformationUtility::CreateTransformation(pickedMatches3D);
		std::vector<unsigned int> candMatchesIndices = std::vector<unsigned int>();
		candMatchesIndices.reserve(matches3D.size());

		double euclideanDistance;

		for (unsigned int matchIndex = 0; matchIndex < matches3D.size(); matchIndex++)
		{
			TransformationUtility::EuclideanDistanceBetweenTwoPoint(euclideanDistance,
				matches3D[matchIndex].trainPair, 
				TransformationUtility::TransformSinglePoint2Point(matches3D[matchIndex].queryPair, *candTransformation));

			if (euclideanDistance < threshold)
			{
				c++;
				candMatchesIndices.push_back(matchIndex);
			}
		}

		if (c > cBest && TransformationUtility::IsTransformationMatrixRightHanded(*candTransformation))
		{
			cBest = c;
			bestMatchesIndices = candMatchesIndices;
		}

		//clear heap
		delete candTransformation;
		delete random3;
	}

	std::vector<Match3D> *result = new std::vector<Match3D>();
	(*result).reserve(bestMatchesIndices.size());

	for (int i = 0; i < bestMatchesIndices.size(); i++)
	{
		(*result).push_back(matches3D[bestMatchesIndices[i]]);
	}

	return result;
}


#pragma region Helpers
//void TransformationUtility::TransformSinglePoint(cv::Matx41f &pt, cv::Matx44f &transformationMatrix)
//{
//	Matx44f transformationT;
//	cv::transpose(transformationMatrix, transformationT);
//	pt = transformationT * pt;	
//}

cv::Matx14f TransformationUtility::TransformSinglePoint2Matrix(cv::Point3f &point, cv::Matx44f &transformationMatrix)
{
	Matx14f pt(point.x, point.y, point.z, 1.0f);
	TransformationUtility::TransformSingleMatrix(pt, transformationMatrix);
	return pt;
}

cv::Point3f TransformationUtility::TransformSinglePoint2Point(cv::Point3f &point, cv::Matx44f &transformationMatrix)
{
	Matx14f pt(point.x, point.y, point.z, 1.0f);
	TransformationUtility::TransformSingleMatrix(pt, transformationMatrix);
	return Point3f(pt(0,0), pt(0,1), pt(0,2));
}

void TransformationUtility::TransformSingleMatrix(cv::Matx14f &pt, cv::Matx44f &transformationMatrix)
{
	pt = pt * transformationMatrix;
}

std::vector<unsigned int> *TransformationUtility::Generate3UniqueRandom(unsigned int ceil)
{
	std::vector<unsigned int> *uniqueNumbers = new std::vector<unsigned int>(3);

	if (ceil >= 3)
	{
		(*uniqueNumbers)[0] = (std::rand() % ceil);

		(*uniqueNumbers)[1] = (std::rand() % (ceil - 1));
		if ((*uniqueNumbers)[0] == (*uniqueNumbers)[1])
		{
			(*uniqueNumbers)[1] += 1;
		}

		(*uniqueNumbers)[2] = (std::rand() % (ceil - 2));
		if ((*uniqueNumbers)[2] == (*uniqueNumbers)[0])
		{
			(*uniqueNumbers)[2] += 1;
		}
		if ((*uniqueNumbers)[2] == (*uniqueNumbers)[1])
		{
			(*uniqueNumbers)[2] =+ 1;
		}
	}
	return uniqueNumbers;
}

bool TransformationUtility::IsTransformationMatrixRightHanded(cv::Matx44f &transformation)
{
	double determinant = cv::determinant(transformation);
	if (determinant > 0.0)
	{
		return true;
	}
	return false;
}

void TransformationUtility::EuclideanDistanceBetweenTwoPoint(double &euclideanDistance, cv::Point3f &pointA, cv::Point3f &pointB)
{
	euclideanDistance = cv::norm(pointA - pointB);
}

#pragma endregion

