#pragma once
#include <opencv/cv.h>
#include <units.h>
#include <vector>



using namespace std;

struct SlicPoint{
    ImCoord position;
    integral_color color;
    SlicPoint() : position(0,0,0), color(0,0,0){}
    SlicPoint(ImCoord p, image_color c) : position(p), color(c){}
    void div(double divisor){
        position[0] /= divisor;
        position[1] /= divisor;
        position[2] /= divisor;
        color[0] /= divisor;
        color[1] /= divisor;
        color[2] /= divisor;
    }
    void add(SlicPoint other){
        position[0] += other.position[0];
        position[1] += other.position[1];
        position[2] += other.position[2];
        color[0] += other.color[0];
        color[1] += other.color[1];
        color[2] += other.color[2];
    }
};

void SlicCluster(cv::Mat image,
                 cv::Mat mask,
                 size_t num_centers,
                 float m,
                 int iterations,
                 vector<SlicPoint>& centers,
                 cv::Mat& clusters);

double SlicDist(SlicPoint a,
                SlicPoint b,
                int region_size,
                float m);

void DrawSegmentation(cv::Mat, const vector<SlicPoint>&, const cv::Mat&);

vector<int_set> FindNeighboringSuperpixels(vector<SlicPoint> centers,
                                           cv::Mat assignments);
