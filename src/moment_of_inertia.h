/**
* moment_of_inertia.h
*/

#pragma once

#include <pclextras/moment_of_inertia_estimation.h>
#include <pclextras/moment_of_inertia_estimation.cpp>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

namespace pDD
{
	class MOIBoundingBox
	{
		public:

			// Constructors for the planeSegmentor
			MOIBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

			// Destructor for planeSegmentor
			~MOIBoundingBox();

			std::vector <float> moment_of_inertia;
			std::vector <float> eccentricity;
			pcl::PointXYZ min_point_AABB;
			pcl::PointXYZ max_point_AABB;
			pcl::PointXYZ min_point_OBB;
			pcl::PointXYZ max_point_OBB;
			pcl::PointXYZ position_OBB;
			Eigen::Matrix3f rotational_matrix_OBB;
			float major_value, middle_value, minor_value;
			Eigen::Vector3f major_vector, middle_vector, minor_vector;
			Eigen::Vector3f mass_center;

			pcl::PointXYZ pt1;
		    pcl::PointXYZ pt2;
		    pcl::PointXYZ pt3;
		    pcl::PointXYZ pt4;
		    pcl::PointXYZ pt5;
		    pcl::PointXYZ pt6;
		    pcl::PointXYZ pt7;
		    pcl::PointXYZ pt8;

		private:

	};
}