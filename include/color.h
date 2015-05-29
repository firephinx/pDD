#pragma once
#define SIMILARITY_THRESH 32
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "units.h"
#include "utils.h"
#include "halton.h"
#include "depth.h"

using namespace std;

int compare_color(const integral_color&, const integral_color&);
int compare_color_exp(const integral_color&, const integral_color&);

short compare_color_diff(int a, int b);
short compare_color_diff_exp(int a, int b);

float color_distance(const integral_color&, const integral_color&);

vector<float> LabColorHistogram(vector<integral_color> colors, int bins);

float CompareHistogram(vector<float> a, vector<float> b);

class HistogramSampler{
    public:
    vector< SpaceCoord > patches_;
    cm patch_radii_;
    
    HistogramSampler(cm dim_x, cm dim_y, int n_patches, cm patch_radii);
    vector<float> ComputeHistogram(const ImCoord& position,
                                   const cv::Mat& integral_image,
                                   const cv::Mat& depth_image,
                                   const Intrinsics& intrinsics,
                                   int bins);
    vector<ImCoord> ValidateHistogramPositions(
            cv::Mat integral_image,
            cv::Mat depth_image,
            Intrinsics intrinsics,
            vector<ImCoord> positions,
            vector<vector<float> > histograms,
            float distance_thresh,
            int bins);
};
