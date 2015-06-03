#pragma once
#include <vector>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <units.h>
#include <camera.h>
#include <yaml_conversion.h>
#include "yaml-cpp/yaml.h"
#include <fstream>

using namespace std;

integral_color_channel getIntegralAverageSquareMono(
        const cv::Mat& integral_image, ImCoord center, px sq_radius);

integral_color getIntegralAverageSquare(
        const cv::Mat& integral_image, ImCoord center, px sq_radius);

/*
void SpaceCoordsToVec3f(
        vector <vector <SpaceCoord> > in, vector< vector< cv::Vec3f> > &out);
*/

void SpatialHomogenize(SpaceCoord& pt);

SpaceCoord homog_cross(SpaceCoord a, SpaceCoord b);

Pose PoseFromVecs(SpaceCoord, SpaceCoord, SpaceCoord, SpaceCoord);

std::string getImageType(int);

void PrintPose(Pose);

void DrawAxes(cv::Mat, Pose, Intrinsics, cm axis_length);

void DrawDebugText(cv::Mat, vector<string>, vector<cv::Scalar>);

Pose MatToPose(cv::Mat);

cv::Mat PoseToMat(Pose pose);

Eigen::Matrix<cm, 4, 4> PoseToMatrix(Pose);

vector<Pose> HaltonOrientationsFromCylinder(int num_orientations,
                                            float max_spin);

Pose OrientationFromCylinderProjection(cm axis, rad angle, rad spin);

SpaceCoord RandomPointInUnitSphere();

Pose RandomOffsetPose(double translation_bound,
                      double rotation_bound,
                      double pivot_bound,
                      SpaceCoord pivot_center,
                      double translation_min_bound=0.0);

vector<Pose> RandomOffsetPoses(int num_poses,
                               double translation_bound,
                               double rotation_bound,
                               double pivot_bound,
                               SpaceCoord pivot_center,
                               double translation_min_bound=0.0);

cv::Mat GetIntegralImage(cv::Mat);

void WritePosesToFile(string path, string key, vector<Pose> poses, int mode);

vector<Pose> ReadPosesFromFile(string path, string key);

SpaceCoord CrossProduct(SpaceCoord a, SpaceCoord b);

Pose TransformAboutLocalPivot(Pose transform, SpaceCoord pivot);

Pose TransformAboutPivot(Pose transform, SpaceCoord pivot);
