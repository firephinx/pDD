#include "moment_of_inertia.h"

using namespace pDD;

MOIBoundingBox::MOIBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud (cloud);
	feature_extractor.compute ();

	feature_extractor.getMomentOfInertia (this->moment_of_inertia);
	feature_extractor.getEccentricity (this->eccentricity);
	feature_extractor.getAABB (this->min_point_AABB, this->max_point_AABB);
	feature_extractor.getOBB (this->min_point_OBB, this->max_point_OBB, this->position_OBB, this->rotational_matrix_OBB);
	feature_extractor.getEigenValues (this->major_value, this->middle_value, this->minor_value);
	feature_extractor.getEigenVectors (this->major_vector, this->middle_vector, this->minor_vector);
	feature_extractor.getMassCenter (this->mass_center);

	Eigen::Vector3f position (this->position_OBB.x, this->position_OBB.y, this->position_OBB.z);
	Eigen::Quaternionf quat (this->rotational_matrix_OBB);

	pcl::PointXYZ center (this->mass_center (0), this->mass_center (1), this->mass_center (2));
	pcl::PointXYZ x_axis (this->major_vector (0) + this->mass_center (0), this->major_vector (1) + this->mass_center (1), this->major_vector (2) + this->mass_center (2));
	pcl::PointXYZ y_axis (this->middle_vector (0) + this->mass_center (0), this->middle_vector (1) + this->mass_center (1), this->middle_vector (2) + this->mass_center (2));
	pcl::PointXYZ z_axis (this->minor_vector (0) + this->mass_center (0), this->minor_vector (1) + this->mass_center (1), this->minor_vector (2) + this->mass_center (2));

	Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

    p1 = rotational_matrix_OBB * p1 + position;
    p2 = rotational_matrix_OBB * p2 + position;
    p3 = rotational_matrix_OBB * p3 + position;
    p4 = rotational_matrix_OBB * p4 + position;
    p5 = rotational_matrix_OBB * p5 + position;
    p6 = rotational_matrix_OBB * p6 + position;
    p7 = rotational_matrix_OBB * p7 + position;
    p8 = rotational_matrix_OBB * p8 + position;

    pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
    pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
    pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
    pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
    pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
    pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
    pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
    pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));

    this->pt1 = pt1;
    this->pt2 = pt2;
    this->pt3 = pt3;
    this->pt4 = pt4;
    this->pt5 = pt5;
    this->pt6 = pt6;
    this->pt7 = pt7;
    this->pt8 = pt8;
}

MOIBoundingBox::~MOIBoundingBox()
{

}