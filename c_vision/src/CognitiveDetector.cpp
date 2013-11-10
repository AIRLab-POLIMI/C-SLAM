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

	HighLevelDetector highLevelDetector;
	highLevelDetector.detect(verticalLines, horizontalLines);

	vector<vector<Point> > rectangles = highLevelDetector.getRectangles();
	vector<vector<Point> > poles = highLevelDetector.getPoles();

	//display results
	viewer.setRoll(roll);
	viewer.setKeyPoints(&keyPoints);
	viewer.setVerticalLines(&verticalLines);
	viewer.setHorizontalLines(&horizontalLines);
	viewer.setClusters(&clusters);
	viewer.setRectangles(&rectangles);
	viewer.setPoles(&poles);
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

