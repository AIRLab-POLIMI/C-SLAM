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

#include "LineDetector.h"
#include "BorderDetector.h"
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

	if (!videoCapture.isOpened())
	{
		std::cout << "Could not open video " << videoPath << std::endl;
		return -1;
	}

	const char* source = "Source Video";
	const char* borderDetection = "Border Detection Video";
	const char* lineDetectionP = "Probabilistic Line Detection video";
	const char* lineDetectionD = "Deterministic Line Detection video";
	const char* cornerDetection = "Corner Detection Video";

	//windows
	namedWindow(source, CV_WINDOW_AUTOSIZE);
	namedWindow(borderDetection, CV_WINDOW_AUTOSIZE);
	namedWindow(lineDetectionP, CV_WINDOW_AUTOSIZE);
	namedWindow(lineDetectionD, CV_WINDOW_AUTOSIZE);
	namedWindow(cornerDetection, CV_WINDOW_AUTOSIZE);

	//Place windows correctly
	cvMoveWindow(source, 0, 0);
	cvMoveWindow(borderDetection, 640, 0);
	cvMoveWindow(lineDetectionP, 0, 480);
	cvMoveWindow(lineDetectionD, 900, 300);
	cvMoveWindow(cornerDetection, 640, 480);

	//instantiate detectors
	BorderDetector borderD(cannyP.minCanny, cannyP.maxCanny, 3, false);
	ProbabilisticHoughLineDetector lineD1(1, CV_PI / 180, houghPP.threshold,
			houghPP.minLineLenght, houghPP.maxLineGap);
	HoughLineDetector lineD2(1, CV_PI / 180, 120);
	CornerDetector cornerD(cornerP.threshold, cornerP.windowSize,
			cornerP.clusterMinSize, cornerP.noiseBarrier, cornerP.objectWindow,
			cornerP.objectMinSize);

	//controls for Canny
	createTrackbar("minCanny", borderDetection, NULL, 300, minCanny,
			(void*) &borderD);
	setTrackbarPos("minCanny", borderDetection, cannyP.minCanny);
	createTrackbar("maxCanny", borderDetection, NULL, 500, maxCanny,
			(void*) &borderD);
	setTrackbarPos("maxCanny", borderDetection, cannyP.maxCanny);

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
	//controls for HoughLineP
	createTrackbar("pThreshold", lineDetectionP, NULL, 150, thresholdHoughP,
			(void*) &lineD1);
	setTrackbarPos("pThreshold", lineDetectionP, houghPP.threshold);
	createTrackbar("minLineLenght", lineDetectionP, NULL, 150,
			minLineLengthHoughP, (void*) &lineD1);
	setTrackbarPos("minLineLenght", lineDetectionP, houghPP.minLineLenght);
	createTrackbar("maxLineGap", lineDetectionP, NULL, 50, maxLineGapHoughP,
			(void*) &lineD1);
	setTrackbarPos("maxLineGap", lineDetectionP, houghPP.maxLineGap);

	//main loop
	while (videoCapture.grab())
	{
		//Variables
		Mat frame;
		Mat borderFrame;
		Mat lineFrameP;
		Mat lineFrameD;
		Mat cornerFrame;

		//get all frames
		videoCapture.retrieve(frame);
		borderFrame = borderD.detect(frame);
		lineFrameP = lineD1.detect(borderFrame);
		lineFrameD = lineD2.detect(borderFrame);
		cornerFrame = cornerD.detect(frame);

		//show all frames
		imshow(source, frame);
		imshow(borderDetection, borderFrame);
		imshow(lineDetectionP, lineFrameP);
		imshow(lineDetectionD, lineFrameD);
		imshow(cornerDetection, cornerFrame);

		cvWaitKey(33);
	}

	return 0;
}

