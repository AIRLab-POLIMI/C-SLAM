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

#include "AdvancedDetector.h"

using namespace cv;

AdvancedDetector::AdvancedDetector(ParameterServer& parameters) :
			BasicDetector(parameters),
			clusterDetector(parameters.getDBScanParams())
{
	clusters = NULL;
}

void AdvancedDetector::detect(Mat& image, Mat& mask)
{
	Mat grayFrame;
	cvtColor(image, grayFrame, CV_BGR2GRAY);

	//detect lines
	detectLines(grayFrame, mask);

	//detect quadrilaterals and poles
	detectQuadrilaterals(true);

	//detect clusters
	clusters = clusterDetector.detect(grayFrame);
}

void AdvancedDetector::deleteDetections()
{
	if(clusters)
	{
		delete clusters;
		clusters = NULL;
	}

	BasicDetector::deleteDetections();
}
