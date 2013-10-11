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

#include "ImageView.h"

#include "Callbacks.h"
#include "DefaultParameters.h"

void ImageView::display(cv::Mat& frame)
{
	displayClusterResults(*keyPoints, *clusters, frame);
	displayLineResults(*verticalLines, frame);
	drawAxis(frame);

	imshow(viewName, frame);
	cvWaitKey(60);
}

void ImageView::drawAxis(cv::Mat& input)
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

void ImageView::displayClusterResults(std::vector<cv::KeyPoint>& keyPoints,
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

void ImageView::displayLineResults(std::vector<cv::Vec4i> lines, cv::Mat& frame)
{

	//display lines
	for (size_t i = 0; i < lines.size(); i++)
	{
		cv::Vec4i l = lines[i];
		line(frame, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
				cv::Scalar(0, 0, 255));
	}

}

void ImageView::createTrackBars(void* featureObject, void* clusterObjetc,
		void* lineObject)
{
	//controls for corners
	cv::createTrackbar("threshold", viewName, NULL, 300, thresholdCorner,
			featureObject);
	cv::setTrackbarPos("threshold", viewName, cornerP.threshold);

	//control for clustering
	cv::createTrackbar("minPoints", viewName, NULL, 20, minPointsCluster,
			clusterObjetc);
	cv::setTrackbarPos("minPoints", viewName, clusterP.minPoints);
	cv::createTrackbar("distance", viewName, NULL, 100, maxDistanceCluster,
			clusterObjetc);
	cv::setTrackbarPos("distance", viewName, clusterP.maxDistance);

	//controls for line
	cv::createTrackbar("pThreshold", viewName, NULL, 150, thresholdHoughP,
			lineObject);
	cv::setTrackbarPos("pThreshold", viewName, houghP.threshold);
	cv::createTrackbar("minLineLenght", viewName, NULL, 150,
			minLineLengthHoughP, lineObject);
	cv::setTrackbarPos("minLineLenght", viewName, houghP.minLineLenght);
	cv::createTrackbar("maxLineGap", viewName, NULL, 50, maxLineGapHoughP,
			lineObject);
	cv::setTrackbarPos("maxLineGap", viewName, houghP.maxLineGap);

}
