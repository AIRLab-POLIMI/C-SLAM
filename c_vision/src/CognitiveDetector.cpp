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
#include "DBSCAN.h"
#include "LineFilter.h"

using namespace cv;
using namespace std;

CognitiveDetector::CognitiveDetector() :
			featureDetector(cornerP.threshold),
			clusterDetector(clusterP.maxDistance, clusterP.minPoints),
			lineDetector(cannyP.apertureSize, houghP.rho, houghP.teta,
						houghP.threshold, houghP.minLineLenght,
						houghP.maxLineGap),
			pitch(0),
			roll(0),
			yaw(0),
			viewer("Detected Image", (void*) &featureDetector,
						(void*) &clusterDetector, (void*) &lineDetector)
{
}

void CognitiveDetector::detect(cv::Mat& frame,
			ObjectClassificator& classificator)
{
	Mat equalizedFrame = preprocessing(frame);
	vector<Vec4i> lines = lineDetector.detect(equalizedFrame);

	LineFilter filter;
	filter.filter(lines, roll);
	const vector<Vec4i>& verticalLines = filter.getVerticalLines();
	const vector<Vec4i>& horizontalLines = filter.getHorizontalLines();

	const vector<KeyPoint>& keyPoints = featureDetector.detect(equalizedFrame);

	const vector<ObjectCluster>& clusters = clusterDetector.detect(keyPoints);

	HighLevelDetector highLevelDetector;
	highLevelDetector.detect(verticalLines, horizontalLines);

	const vector<vector<Point> >& rectangles =
				highLevelDetector.getRectangles();
	const vector<vector<Point> >& poles = highLevelDetector.getPoles();

	//display results
	viewer.setRoll(roll);
	viewer.setKeyPoints(&keyPoints);
	viewer.setVerticalLines(&verticalLines);
	viewer.setHorizontalLines(&horizontalLines);
	viewer.setClusters(&clusters);
	viewer.setRectangles(&rectangles);
	viewer.setPoles(&poles);
	viewer.display(frame);

	//send results to the reasoner
	processRectangles(rectangles, classificator);
	processPoles(poles, classificator);
	processClusters(clusters, classificator);
}

void CognitiveDetector::processRectangles(
			const vector<vector<Point> >& rectangles,
			ObjectClassificator& classificator)
{
	for (vector<vector<Point> >::const_iterator i = rectangles.begin();
				i != rectangles.end(); ++i)
	{
		classificator.newObject();
		int xMin, xMax, yMin, yMax;
		int FormFactor;

		vector<Point>::const_iterator j = i->begin();
		xMax = xMin = j->x;
		yMin = yMax = j->y;

		j++;

		for (; j != i->end(); ++j)
		{
			xMax = max(j->x, xMax);
			yMax = max(j->y, yMax);
			xMin = min(j->x, xMin);
			yMin = min(j->y, yMin);
		}

		int deltaX = (xMax - xMin);
		int deltaY = (yMax - yMin);
		if (deltaY != 0)
		{
			FormFactor = 1000 * deltaX / deltaY;
			classificator.addFeature("xMax", xMax);
			classificator.addFeature("yMax", yMax);
			classificator.addFeature("xMin", xMin);
			classificator.addFeature("yMin", yMin);
			classificator.addFeature("formFactor", FormFactor);
			classificator.addFeature("color", 100); //FIXME levami!!!!
		}

	}
}

void CognitiveDetector::processPoles(const vector<vector<Point> >& poles,
			ObjectClassificator& classificator)
{
	for (vector<vector<Point> >::const_iterator it = poles.begin();
				it != poles.end(); ++it)
	{

	}
}

void CognitiveDetector::processClusters(const vector<ObjectCluster>& clusters,
			ObjectClassificator& classificator)
{

}

/* Image Processing Methods */

Mat CognitiveDetector::preprocessing(Mat& input)
{
	Mat greyFrame, equalizedFrame;

	cvtColor(input, greyFrame, CV_BGR2GRAY);
	equalizeHist(greyFrame, equalizedFrame);

	return equalizedFrame;
}

