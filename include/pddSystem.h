/**
* pddSystem.h
*/

#pragma once
#include <vector>
#include <boost/thread.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/locks.hpp>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include "settings.h"

namespace pDD
{
	//class trackingPlanes;
	//class Frame;

	class pddSystem
	{
		public:

			// Settings. Constant from construction onward.
			int width;
			int height;
			//const bool PDDEnabled;

			//bool trackingIsGood:

			// Constructors for pddSystem
			pddSystem(cv_bridge::CvImagePtr RGBImg, cv_bridge::CvImagePtr DepthImg);

			// Destructor for pddSystem
			~pddSystem();

			// tracks a frame.
			// first frame will return Identity = camToWord.
			// returns camToWord transformation of the tracked frame.
			// frameID needs to be monotonically increasing.
			//void trackFrame(uchar* image, unsigned int frameID, bool blockUntilMapped, double timestamp);

		private:

			//TrackingReference* trackingReference;
	};
}