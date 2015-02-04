/*
 * c_vision,
 *
 *
 * Copyright (C) 2015 Davide Tateo
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

#include "CornerClassifier.h"

#include <angles/angles.h>

using namespace std;
using namespace cv;

const vector<Mat> CornerClassifier::cornerPrototypes(CornerClassifier::initPrototypes());

const size_t CornerClassifier::prototypeN =
			CornerClassifier::cornerPrototypes.size();

CornerClassifier::CornerClassifier(CornerClassParam& params, const Mat& canny,
			double roll) :
			params(params), canny(canny)
{
	sinR = std::sin(-roll);
	cosR = std::cos(-roll);
}

bool CornerClassifier::isCompatibleCorner(const Point& point, CornerType type)
{
	Mat hist;
	computeHistogram(point, hist);
	CornerType classifiedCorner = findNearestNeighbour(hist);
	bool same = classifiedCorner == type || classifiedCorner == NSWE;

	switch (type)
	{
		case NW:
			return same || classifiedCorner == NWE || classifiedCorner == NSW;

		case NE:
			return same || classifiedCorner == NWE || classifiedCorner == NSE;

		case SW:
			return same || classifiedCorner == SWE || classifiedCorner == NSW;

		case SE:
			return same || classifiedCorner == SWE || classifiedCorner == NSE;

		default:
			return same;

	}

}

void CornerClassifier::computeHistogram(const Point& point, Mat& hist)
{
	int minX = max(0, point.x - params.kernelSize);
	int maxX = min(canny.cols, point.x + params.kernelSize);
	int minY = max(0, point.y - params.kernelSize);
	int maxY = min(canny.rows, point.y + params.kernelSize);
	Point minP(minX, minY);

	Mat roi = canny(Range(minY, maxY), Range(minX, maxX));
	Mat histPlain = Mat(1, 4, CV_32FC1, Scalar(0.0));
	hist = Mat(1, 4, CV_32FC1);

	for (int i = 0; i < roi.rows; ++i)
	{
		uchar* p = roi.ptr<uchar>(i);
		for (int j = 0; j < roi.cols; ++j)
		{
			if (p[j])
				addToBucket(point, minP, Point(j, i), histPlain);
		}
	}

	normalize(histPlain, hist);
}

void CornerClassifier::addToBucket(const Point& c, const Point& min, const Point& curr, Mat& hist)
{
	//get params
	const int w = params.bucketWidth;
	const int k = params.kernelSize;

	//Rotate coordinates
	int dxo = curr.x + min.x - c.x;
	int dyo = curr.y + min.y - c.y;

	int dx = dxo * cosR - dyo * sinR;
	int dy = dxo * sinR + dyo * cosR;

	//add to correct bucket
	if (abs(dx) <= w)
	{
		if (dy >= -k && dy <= -w)
		{
			hist.at<uchar>(0, N) += 1.0;
		}
		else if (dy >= w && dy <= k)
		{
			hist.at<uchar>(0, S) += 1.0;
		}
	}
	else if (abs(dy) <= w)
	{
		if (dx >= -k && dx <= -w)
		{
			hist.at<uchar>(0, W) += 1.0;
		}
		else if (dx >= w && dx <= k)
		{
			hist.at<uchar>(0, E) += 1.0;
		}
	}

}

CornerType CornerClassifier::findNearestNeighbour(const Mat& hist)
{
	cout << hist << endl;
	vector<double> distances(prototypeN);
	for (size_t i = 0; i < cornerPrototypes.size(); i++)
	{
		distances[i] = compareHist(hist, cornerPrototypes[i],
					CV_COMP_BHATTACHARYYA);
	}

	return static_cast<CornerType>(distance(distances.begin(),
				min_element(distances.begin(), distances.end())));
}

vector<Mat> CornerClassifier::initPrototypes()
{
	vector<Mat> prototypes;
	prototypes.resize(NSWE + 1);

	// L corners
	float dataNW[4] =
	{ 1.0 / 2, 0, 1.0 / 2, 0 };
	prototypes[NW] = Mat(1, 4, CV_32FC1, dataNW).clone();

	float dataNE[4] =
	{ 1.0 / 2, 0, 0, 1.0 / 2 };
	prototypes[NE] = Mat(1, 4, CV_32FC1, dataNE).clone();

	float dataSW[4] =
	{ 0, 1.0 / 2, 1.0 / 2, 0 };
	prototypes[SW] = Mat(1, 4, CV_32FC1, dataSW).clone();

	float dataSE[4] =
	{ 0, 1.0 / 2, 0, 1.0 / 2 };
	prototypes[SE] = Mat(1, 4, CV_32FC1, dataSE).clone();

	// I corners
	float dataNS[4] =
	{ 1.0 / 2, 1.0 / 2, 0, 0 };
	prototypes[NS] = Mat(1, 4, CV_32FC1, dataNS).clone();

	float dataWE[4] =
	{ 0, 0, 1.0 / 2, 1.0 / 2 };
	prototypes[WE] = Mat(1, 4, CV_32FC1, dataWE).clone();

	// T corners
	float dataNWE[4] =
	{ 1.0 / 3, 0, 1.0 / 3, 1.0 / 3 };
	prototypes[NWE] = Mat(1, 4, CV_32FC1, dataNWE).clone();

	float dataSWE[4] =
	{ 0, 1.0 / 3, 1.0 / 3, 1.0 / 3 };
	prototypes[SWE] = Mat(1, 4, CV_32FC1, dataSWE).clone();

	float dataNSW[4] =
	{ 1.0 / 3, 1.0 / 3, 1.0 / 3, 0 };
	prototypes[NSW] = Mat(1, 4, CV_32FC1, dataNSW).clone();

	float dataNSE[4] =
	{ 1.0 / 3, 1.0 / 3, 0, 1.0 / 3 };
	prototypes[NSE] = Mat(1, 4, CV_32FC1, dataNSE).clone();

	// X corners
	float dataNSWE[4] =
	{ 1.0 / 4, 1.0 / 4, 1.0 / 4, 1.0 / 4 };
	prototypes[NSWE] = Mat(1, 4, CV_32FC1, dataNSWE).clone();

	return prototypes;

}
