#pragma once
#include <vector>
#include "units.h"
#include "utils.h"
#include "model.h"
#include "camera.h"
#include "halton.h"
#include "binomial.h"
#include "color.h"

vector<Pose> DepthScan(const cv::Mat& integral_image,
                       const cv::Mat& depth_image,
                       const Intrinsics& intrinsics,
                       const Model& model,
                       int layer,
                       const BinomialInferenceModel& depth_binomial,
                       const BinomialInferenceModel& color_binomial,
                       const vector<ImCoord>& positions,
                       const vector<Pose>& orientations,
                       Pose gt_pose,
                       vector<float>& probs);


vector<Pose> OrientationScan(const cv::Mat& integral_image,
                             const cv::Mat& depth_image,
                             const Intrinsics& intrinsics,
                             const Model& model,
                             int layer,
                             const BinomialInferenceModel& depth_binomial,
                             const BinomialInferenceModel& color_binomial,
                             const vector<Pose>& orientations,
                             Pose gt_pose,
                             vector<float>& probs);
