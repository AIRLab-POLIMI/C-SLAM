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

#include <opencv2/imgproc/imgproc.hpp>

#include "CognitiveDetector.h"
#include "Callbacks.h"
#include "DBScan.h"

void CognitiveDetector::detect(cv::Mat frame)
{

	cv::Mat lineFrame = frame.clone();
	std::vector<cv::Vec4i> lines = lineDetector.detect(frame);
	std::vector<cv::Vec4i> verticalLines = extractVerticalLines(lines);

	std::vector<cv::KeyPoint> keyPoints = featureDetector.detect(frame);

	std::vector<ObjectCluster> clusters = clusterDetector.detect(keyPoints);

	//display results
	displayClusterResults(keyPoints, clusters, frame);
	displayLineResults(lines, lineFrame);
	drawAxis(frame);

	imshow(CORNER_WINDOW, frame);
	imshow(LINE_WINDOW, lineFrame);
	cvWaitKey(60);
}

void CognitiveDetector::drawAxis(cv::Mat& input)
{
	cv::Point center, y1, y2, z1, z2;
	center.x = input.cols / 2;
	center.y = input.rows / 2;

	double offset1 = 50 * sin(roll * M_PI / 180.0);
	double offset2 = 50 * cos(roll * M_PI / 180.0);

	y1.x = center.x + offset2;
	y1.y = center.y - offset1;
	y2.x = center.x - offset2;
	y2.y = center.y + offset1;

	z1.x = center.x - offset1;
	z1.y = center.y - offset2;
	z2.x = center.x + offset1;
	z2.y = center.y + offset2;

	cv::line(input, y1, y2, cv::Scalar(0, 0, 0));
	cv::line(input, z1, z2, cv::Scalar(0, 0, 0));
	cv::circle(input, center, 5, cv::Scalar(0, 0, 0));

}

void CognitiveDetector::displayClusterResults(
		std::vector<cv::KeyPoint>& keyPoints,
		std::vector<ObjectCluster>& clusters, cv::Mat& frame)
{
	//display results
	for (size_t i = 0; i < keyPoints.size(); ++i)
	{
		const cv::KeyPoint& kp = keyPoints[i];
		cv::circle(frame, kp.pt, 2, cv::Scalar(0, 0, 255));
	}

	/*display clusters*/
	for (size_t i = 0; i < clusters.size(); i++)
	{

		clusters[i].draw(frame, cv::Scalar(255, 0, 0));
	}
}

void CognitiveDetector::displayLineResults(std::vector<cv::Vec4i> lines, cv::Mat& frame)
{

	//display lines
	for (size_t i = 0; i < lines.size(); i++)
	{
		cv::Vec4i l = lines[i];
		line(frame, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
				cv::Scalar(0, 0, 255), 3, 8);
	}

}

void CognitiveDetector::extractVerticalLines(std::vector<cv::Vec4i> lines)
{

	double m_robot = tan(roll * M_PI / 180.0);
	for(size_t i; i < lines.size(); i++)
	{
		cv::Vec4i& line = lines[i];
		double m_line = (line[0] - line[2]) / (line[1] - line[3]);

		if((m_robot - m_line < 0.1) && (m_robot - m_line > 0.1))
		{


		}
	}

}

void CognitiveDetector::createTrackBars()
{
	//controls for corners
	cv::createTrackbar("threshold", CORNER_WINDOW, NULL, 300, thresholdCorner,
			(void*) &featureDetector);
	cv::setTrackbarPos("threshold", CORNER_WINDOW, cornerP.threshold);

	//control for clustering
	cv::createTrackbar("minPoints", CORNER_WINDOW, NULL, 20, minPointsCluster,
			(void*) &clusterDetector);
	cv::setTrackbarPos("minPoints", CORNER_WINDOW, clusterP.minPoints);
	cv::createTrackbar("distance", CORNER_WINDOW, NULL, 100, maxDistanceCluster,
			(void*) &clusterDetector);
	cv::setTrackbarPos("distance", CORNER_WINDOW, clusterP.maxDistance);

	//controls for line
	cv::createTrackbar("minCanny", LINE_WINDOW, NULL, 300, minCanny,
			(void*) &lineDetector);
	cv::setTrackbarPos("minCanny", LINE_WINDOW, cannyP.minCanny);
	cv::createTrackbar("maxCanny", LINE_WINDOW, NULL, 500, maxCanny,
			(void*) &lineDetector);
	cv::setTrackbarPos("maxCanny", LINE_WINDOW, cannyP.maxCanny);
	cv::createTrackbar("pThreshold", LINE_WINDOW, NULL, 150, thresholdHoughP,
			(void*) &lineDetector);
	cv::setTrackbarPos("pThreshold", LINE_WINDOW, houghP.threshold);
	cv::createTrackbar("minLineLenght", LINE_WINDOW, NULL, 150,
			minLineLengthHoughP, (void*) &lineDetector);
	cv::setTrackbarPos("minLineLenght", LINE_WINDOW, houghP.minLineLenght);
	cv::createTrackbar("maxLineGap", LINE_WINDOW, NULL, 50, maxLineGapHoughP,
			(void*) &lineDetector);
	cv::setTrackbarPos("maxLineGap", LINE_WINDOW, houghP.maxLineGap);

}
