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
#include "DBScan.h"
#include "LineFilter.h"

using namespace cv;
using namespace std;

void CognitiveDetector::detect(cv::Mat& frame)
{
	Mat equalizedFrame = preprocessing(frame);
	vector<Vec4i> lines = lineDetector.detect(equalizedFrame);

	LineFilter filter;
	filter.filter(lines, roll);
	vector<Vec4i> verticalLines = filter.getVerticalLines();
	vector<Vec4i> horizontalLines = filter.getHorizontalLines();

	vector<KeyPoint> keyPoints = featureDetector.detect(equalizedFrame);

	vector<ObjectCluster> clusters = clusterDetector.detect(keyPoints);
	vector<vector<Point> > squares = detectSquares(verticalLines,
			horizontalLines);

	//display results
	viewer.setRoll(roll);
	viewer.setKeyPoints(&keyPoints);
	viewer.setVerticalLines(&verticalLines);
	viewer.setHorizontalLines(&horizontalLines);
	viewer.setClusters(&clusters);
	viewer.setSquares(&squares);
	viewer.display(frame);
}

/* Image Processing Methods */

Mat CognitiveDetector::preprocessing(Mat& input)
{
	Mat greyFrame, equalizedFrame;

	cvtColor(input, greyFrame, CV_BGR2GRAY);
	equalizeHist(greyFrame, equalizedFrame);

	return equalizedFrame;
}

/* square detection algorithm */
vector<vector<Point> > CognitiveDetector::detectSquares(
		std::vector<cv::Vec4i> verticalLines,
		std::vector<cv::Vec4i> horizontalLines)
{
	vector<vector<Point> > squares;
	for (int i = 0; i < verticalLines.size() - 1; i++)
	{
		Vec4i v1 = verticalLines[i];
		Vec4i v2 = verticalLines[i + 1];
		for (int j = 0; j < horizontalLines.size() - 1; j++)
		{
			Vec4i h1 = horizontalLines[j];
			for (int k = j + 1; k < horizontalLines.size(); k++)
			{
				vector<Point> square;
				Vec4i h2 = horizontalLines[k];
				Point a, b, c, d;
				a = findInterceptions(h1, v1);
				b = findInterceptions(h1, v2);
				c = findInterceptions(h2, v1);
				d = findInterceptions(h2, v2);
				square.push_back(a);
				square.push_back(b);
				square.push_back(c);
				square.push_back(d);

				squares.push_back(square);
			}
		}

	}

	return squares;

}

Point CognitiveDetector::findInterceptions(Vec4i l1, Vec4i l2)
{
	int x1 = l1[0], x2 = l1[2];
	int y1 = l1[1], y2 = l1[3];
	int x3 = l2[0], x4 = l2[2];
	int y3 = l2[1], y4 = l2[3];

	double a =
			-(x2 * (y4 - y3) + x3 * (y2 - y4) + x4 * (y3 - y2))
					/ (x1 * (y4 - y3) + x2 * (y3 - y4) + x4 * (y2 - y1)
							+ x3 * (y1 - y2));
	double b =
			(x1 * (y4 - y2) + x2 * (y1 - y4) + x4 * (y2 - y1))
					/ (x1 * (y4 - y3) + x2 * (y3 - y4) + x4 * (y2 - y1)
							+ x3 * (y1 - y2));
	int x = a * x1 + (1 - a) * x2;
	int y = a * y1 + (1 - a) * y2;

	return Point(x, y);

}

