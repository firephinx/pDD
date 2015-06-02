/**
* pddSystem.cpp
*/

#include <vector>
#include <numeric>
#include "pddSystem.h"

#include "opencv2/opencv.hpp"
#include "utils.h"

using namespace pDD;

pddSystem::pddSystem(cv_bridge::CvImagePtr RGBImg, cv_bridge::CvImagePtr DepthImg)
{
	this->width = RGBImg->image.cols;
	this->height = RGBImg->image.rows;
	printf(this->width);
	printf(this->height);
}

pddSystem::~pddSystem()
{

}
