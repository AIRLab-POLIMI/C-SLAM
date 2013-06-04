/*
 * c_vision,
 *
 *
 * Copyright (C) 2013 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_vision.
 *
 * c_vision is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_vision is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_vision.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * c_vision,
 *
 *
 * Copyright (C) 2013 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_vision.
 *
 * c_vision is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_vision is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_vision.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "CornerDetector.h"

#include "Callbacks.h"
#include "DefaultParameters.h"

using namespace cv;

int main(int argc, char *argv[])
{
	if (argc != 2)
		std::cout << "Usage: ./example <your video>" << std::endl;

	const std::string videoPath = argv[1];

	VideoCapture videoCapture(videoPath);
	Mat sizeFrame;
	videoCapture >> sizeFrame;
	VideoWriter videoWriter("/home/dave/output.avi",
			CV_FOURCC('D', 'I', 'V', 'X'), 25, sizeFrame.size(), true);

	if (!videoCapture.isOpened())
	{
		std::cout << "Could not open video " << videoPath << std::endl;
		return -1;
	}

	const char* source = "Source Video";

	const char* cornerDetection = "Corner Detection Video";

	//windows
	namedWindow(source, CV_WINDOW_AUTOSIZE);

	namedWindow(cornerDetection, CV_WINDOW_AUTOSIZE);

	//Place windows correctly
	cvMoveWindow(source, 0, 0);
	cvMoveWindow(cornerDetection, 640, 480);

	//instantiate detectors
	CornerDetector cornerD(cornerP.threshold, cornerP.windowSize,
			cornerP.clusterMinSize, cornerP.noiseBarrier, cornerP.objectWindow,
			cornerP.objectMinSize);

	//Controls for corner
	createTrackbar("threshold", cornerDetection, NULL, 50, thresholdCorner,
			(void*) &cornerD);
	setTrackbarPos("threshold", cornerDetection, cornerP.threshold);
	createTrackbar("windowSize", cornerDetection, NULL, 15, windowSizeCorner,
			(void*) &cornerD);
	setTrackbarPos("windowSize", cornerDetection, cornerP.windowSize);
	createTrackbar("minSize", cornerDetection, NULL, 350, minClusterSizeCorner,
			(void*) &cornerD);
	setTrackbarPos("minSize", cornerDetection, cornerP.clusterMinSize);
	createTrackbar("noiseBarrier", cornerDetection, NULL, 350,
			noisebarrierCorner, (void*) &cornerD);
	setTrackbarPos("noiseBarrier", cornerDetection, cornerP.noiseBarrier);
	createTrackbar("objectWindow", cornerDetection, NULL, 350,
			objectWindoCorner, (void*) &cornerD);
	setTrackbarPos("objectWindow", cornerDetection, cornerP.objectWindow);
	createTrackbar("minObjectSize", cornerDetection, NULL, 350,
			objectMinSizeCorner, (void*) &cornerD);
	setTrackbarPos("minObjectSize", cornerDetection, cornerP.objectMinSize);

	//main loop
	while (videoCapture.grab())
	{
		//Variables
		Mat frame;
		Mat cornerFrame;

		//get all frames
		videoCapture.retrieve(frame);
		cornerFrame = cornerD.detect(frame);

		videoWriter << cornerFrame;

		//show all frames
		imshow(source, frame);
		imshow(cornerDetection, cornerFrame);

		cvWaitKey(33);
	}

	return 0;
}

