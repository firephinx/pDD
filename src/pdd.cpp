/**
* pdd.cpp
*/

#include "pdd.h"
#include <vector>

#include "pddSystem.h"
#include "IOWrapper/ImageDisplay.h"
#include "IOWrapper/Output3DWrapper.h"
#include "IOWrapper/InputImageStream.h"
#include "util/glocalFunctions.h"

#include <iostream>

namespace pDD {

	pdd::pdd(InputImageStream* imageStream, Output3DWrapper* outputWrapper)
	{
		this->imageStream = imageStream;
		this->outputWrapper = outputWrapper;
		imageStream->getBuffer()->setReceiver(this);

		fx = imageStream->fx();
		fy = imageStream->fy();
		cx = imageStream->cx();
		cy = imageStream->cy();
		width = imageStream->width();
		height = imageStream->height();

		isInitialized = false;

		pddS = new pddSystem(width,height, doPDD);
		pddS->setVisualization(outputWrapper);

		imageSeqNumber = 0;
	}

	pdd::~pdd()
	{
		if(pddS != 0)
			delete pddS;
	}

	void pdd::Loop()
	{
		while (true) {
			boost::unique_lock<boost::recursive_mutex> waitLock(imageStream->getBuffer()->getMutex());
			while (!fullResetRequested && !(imageStream->getBuffer()->size() > 0)) {
				notifyCondition.wait(waitLock);
			}
			waitLock.unlock();
			
			
			if(fullResetRequested)
			{
				resetAll();
				fullResetRequested = false;
				if (!(imageStream->getBuffer()->size() > 0))
					continue;
			}
			
			TimestampedMat image = imageStream->getBuffer()->first();
			imageStream->getBuffer()->popFront();
			
			// process image
			//Util::displayImage("MyVideo", image.data);
			newImageCallback(image.data, image.timestamp);
		}
	}

	void pdd::newImageCallback(const cv::Mat& img, Timestamp imgTime)
	{ 
		++ imageSeqNumber;

		// Convert image to grayscale, if necessary
		cv::Mat grayImg;
		if (img.channels() == 1)
			grayImg = img;
		else
			cvtColor(img, grayImg, CV_RGB2GRAY);
		

		// Assert that we work with 8 bit images
		assert(grayImg.elemSize() == 1);
		assert(fx != 0 || fy != 0);


		// need to initialize
		if(!isInitialized)
		{
			pddS->randomInit(grayImg.data, imgTime.toSec(), 1);
			isInitialized = true;
		}
		else if(isInitialized && pddS != nullptr)
		{
			pddS->trackFrame(grayImg.data,imageSeqNumber,false,imgTime.toSec());
		}
	}

	void pdd::requestReset()
	{
		fullResetRequested = true;
		notifyCondition.notify_all();
	}

	void pdd::resetAll()
	{
		if(pddS != nullptr)
		{
			delete pddS;
			printf("Deleted PDDSystem Object!\n");

			pddS = new pddSystem(width,height,K, doSlam);
			pddS->setVisualization(outputWrapper);

		}
		imageSeqNumber = 0;
		isInitialized = false;

		Util::closeAllWindows();
	}
}
