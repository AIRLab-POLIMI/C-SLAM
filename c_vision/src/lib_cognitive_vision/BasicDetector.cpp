/*
 * c_vision,
 *
 *
 * Copyright (C) 2014 Davide Tateo
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

#include "BasicDetector.h"

#include <opencv2/imgproc/imgproc.hpp>
#include "QuadrilateralDetector.h"
#include "CornerClassifier.h"

using namespace cv;

BasicDetector::BasicDetector(ParameterServer& parameters) :
			lineDetector(parameters.getCannyParams(),
						parameters.getHoughParams(),
						parameters.getLFiltrerParams()),
			quadParams(parameters.getQuadDetectorParams()),
			cornerParams(parameters.getCornerClassParams())
{
	horizontalLines = NULL;
	verticalLines = NULL;
	rectangles = NULL;
	poles = NULL;
	roll = 0;
}

void BasicDetector::deleteDetections()
{
	if (verticalLines && horizontalLines)
	{
		delete verticalLines;
		delete horizontalLines;

		verticalLines = NULL;
		horizontalLines = NULL;
	}

	if (rectangles && poles)
	{
		delete rectangles;
		delete poles;

		rectangles = NULL;
		poles = NULL;
	}
}

void BasicDetector::detectLines(Mat& image, const Mat& mask, bool showCanny)
{
	lineDetector.detect(image, roll, mask);

	if(showCanny)
		lineDetector.display();

	verticalLines = lineDetector.getVerticalLines();
	horizontalLines = lineDetector.getHorizontalLines();
}

void BasicDetector::detectQuadrilaterals(bool skipCheck)
{
	CornerClassifier cornerClassifier(cornerParams, lineDetector.getCanny(), roll);
	QuadrilateralDetector quadrilateralDetector(quadParams, cornerClassifier);
	quadrilateralDetector.detect(*verticalLines, *horizontalLines, true);
	rectangles = quadrilateralDetector.getRectangles();
	poles = quadrilateralDetector.getPoles();
}

BasicDetector::~BasicDetector()
{
}
