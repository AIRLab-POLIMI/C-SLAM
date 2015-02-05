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

const vector<Mat> CornerClassifier::cornerPrototypes(
			CornerClassifier::initPrototypes());

ostream &operator<<(ostream& output, const CornerType& type)
{

	switch (type)
	{
		case NW:
			output << "NW";
			break;
		case NE:
			output << "NE";
			break;
		case SW:
			output << "SW";
			break;
		case SE:
			output << "SE";
			break;
		case N:
			output << "N";
			break;
		case S:
			output << "S";
			break;
		case W:
			output << "W";
			break;
		case E:
			output << "E";
			break;
		case NS:
			output << "NS";
			break;
		case WE:
			output << "WE";
			break;
		case NWE:
			output << "NWE";
			break;
		case SWE:
			output << "SWE";
			break;
		case NSW:
			output << "NSW";
			break;
		case NSE:
			output << "NSE";
			break;
		case EMPTY:
			output << "EMPTY";
			break;
		case NSWE:
			output << "NSWE";
			break;
	}

	return output;
}

CornerResult::CornerResult(CornerType type) :
			type(type)
{

}

bool CornerResult::isCompatible(CornerType type)
{
	bool same = this->type == type || this->type == NSWE || this->type == EMPTY;

	switch (type)
	{
		case NW:
			return same || this->type == NWE || this->type == NSW;

		case NE:
			return same || this->type == NWE || this->type == NSE;

		case SW:
			return same || this->type == SWE || this->type == NSW;

		case SE:
			return same || this->type == SWE || this->type == NSE;

		default:
			return same;

	}
}

CornerClassifier::CornerClassifier(CornerClassParam& params, const Mat& canny,
			double roll) :
			params(params), canny(canny)
{
	//TODO check if the roll sign id correct
	sinR = std::sin(roll);
	cosR = std::cos(roll);
}

CornerResult CornerClassifier::getResult(const Point& point)
{
	Mat hist;
	computeHistogram(point, hist);
	CornerType type = findNearestNeighbour(hist);

	return CornerResult(type);
}

void CornerClassifier::computeHistogram(const Point& point, Mat& hist)
{
	int minX = max(0, point.x - params.kernelSize);
	int maxX = min(canny.cols, point.x + params.kernelSize);
	int minY = max(0, point.y - params.kernelSize);
	int maxY = min(canny.rows, point.y + params.kernelSize);

	Mat histPlain = Mat(1, 4, CV_32FC1, Scalar(0.0));
	hist = Mat(1, 4, CV_32FC1);

	for (int i = minY; i < maxY; ++i)
	{
		const uchar* p = canny.ptr<uchar>(i);
		for (int j = minX; j < maxX; ++j)
		{
			if (p[j])
				addToBucket(point, Point(j, i), histPlain);
		}
	}

	normalize(histPlain, hist, 1.0, 0.0, NORM_L1);
}

void CornerClassifier::addToBucket(const Point& c, const Point& current,
			Mat& hist)
{
	//get params
	const int w = params.bucketWidth;
	const int k = params.kernelSize;

	//Rotate coordinates
	int dxo = current.x - c.x;
	int dyo = current.y - c.y;

	int dx = dxo * cosR - dyo * sinR;
	int dy = dxo * sinR + dyo * cosR;

	//add to correct bucket
	if (abs(dx) <= w)
	{
		if (dy >= -k && dy <= -w)
		{
			hist.at<float>(0, NORTH) += 1.0;
		}
		else if (dy >= w && dy <= k)
		{
			hist.at<float>(0, SOUTH) += 1.0;
		}
	}
	else if (abs(dy) <= w)
	{
		if (dx >= -k && dx <= -w)
		{
			hist.at<float>(0, WEST) += 1.0;
		}
		else if (dx >= w && dx <= k)
		{
			hist.at<float>(0, EAST) += 1.0;
		}
	}

}

CornerType CornerClassifier::findNearestNeighbour(const Mat& hist)
{
	if (norm(hist, NORM_L1) == 0)
		return EMPTY;

	vector<double> distances(cornerPrototypes.size());
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
	float dataN[4] =
	{ 1, 0, 0, 0 };
	prototypes[N] = Mat(1, 4, CV_32FC1, dataN).clone();

	float dataS[4] =
	{ 0, 1, 0, 0 };
	prototypes[S] = Mat(1, 4, CV_32FC1, dataS).clone();

	float dataW[4] =
	{ 0, 0, 1, 0 };
	prototypes[W] = Mat(1, 4, CV_32FC1, dataW).clone();

	float dataE[4] =
	{0, 0, 0, 1 };
	prototypes[E] = Mat(1, 4, CV_32FC1, dataE).clone();

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
