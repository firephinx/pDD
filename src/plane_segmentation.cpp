#include "plane_segmentation.h"

using namespace pDD;

planeSegmentor::planeSegmentor(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);
  seg.setMaxIterations (1000);
  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_THROW_EXCEPTION (pcl::PCLException, "Could not estimate a planar model for the given dataset.");
  }

  // Change the sign of all coefficients so that "d" in ax+by+cz+d=0 is always positive
  /*float signOfD = (coefficients->values[3]) / abs(coefficients->values[3]);
  coefficients->values[0] = coefficients->values[0] * signOfD;
  coefficients->values[1] = coefficients->values[1] * signOfD;
  coefficients->values[2] = coefficients->values[2] * signOfD;
  coefficients->values[3] = coefficients->values[3] * signOfD;*/

  this->MC = coefficients;
  this->PI = inliers;

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Number of inliers : " << inliers->indices.size() << std::endl;

}


planeSegmentor::~planeSegmentor()
{

}

  