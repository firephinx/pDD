/**
* pdd.h
*/

#pragma once

#include <iostream>
#include <fstream>
#include <chrono>

#include "IOWrapper/Timestamp.h"
#include "IOWrapper/NotifyBuffer.h"
#include "IOWrapper/TimestampedObject.h"

#include "util/SophusUtil.h"  /**  Don't really know what this is for yet */

namespace cv {
	class Mat;
}

namespace pDD {
	class pddSystem;
	class InputImageStream;
	class Output3DWrapper;

	struct pdd : public Notifiable
	{
		friend class pddROS;
		public:

			pdd(InputImageStream* imageStream, Output3DWrapper* outputWrapper);

			/** Destructor */
			~pdd();

			/** Runs the main processing loop. Will never return. */
			void Loop();

			/** Requests a reset from a different thread. */
			void requestReset();

			/** Resets everything */
			void resetAll();

			/** Callback function for new RGB and depth images */
			void newImageCallback(const cv::Mat& img, Timestamp imgTime);

			inline pddSystem* getPDDSystem() {return pddS;}

		private:

			InputImageStream* imageStream;
			Output3DWrapper* outputWrapper;

			//initialization flag
			bool isInitialized

			//pddSystem values
			pddSystem* pddS;

			float fx, fy, cx, cy;
			int width, height;

			int imageSeqNumber;

	};
}