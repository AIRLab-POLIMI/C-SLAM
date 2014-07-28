/*
 * c_tracking,
 *
 *
 * Copyright (C) 2014 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_tracking.
 *
 * c_tracking is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_tracking is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_tracking.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "MappingTracker.h"

using namespace std;
using namespace cv;

MappingTracker::MappingTracker()
{
	objectMapped = false;

	minDistance = 120.0;
}

void MappingTracker::initialize(const Mat& im_gray0, InitializationData& data)
{
	mappedKeyPoints = data.selected_keypoints;
	CMT::initialize(im_gray0, data);
}

void MappingTracker::mapObject(Mat& image, const Mat_<double>& K)
{
	if (!objectMapped)
	{
		const vector<pair<KeyPoint, int> >& matches =
					this->getActiveKeypoints();

		size_t size = matches.size();
		double averageDistance = 0;

		if (size > 8)
		{
			vector<Point2f> points1;
			vector<Point2f> points2;

			for (int i = 0; i < size; i++)
			{
				int matchIndex = matches[i].second;
				const Point2f& matchPoint = matches[i].first.pt;
				const Point2f& basePoint = mappedKeyPoints[matchIndex].pt;

				points1.push_back(basePoint);
				points2.push_back(matchPoint);

				line(image, matchPoint, basePoint, Scalar(0, 0, 255));

				averageDistance += norm(matchPoint - basePoint);
			}

			averageDistance /= size;

			if (averageDistance > minDistance)
			{

				//TODO mapping... or save data to do mapping...
				Mat F = findFundamentalMat(points1, points2);
				Mat E = K.t() * F * K;
				objectMapped = true;
			}

		}

	}
}

//FIXME Opencv3 function, to delete when opencv3 will be relased
void MappingTracker::decomposeEssentialMat(InputArray _E, OutputArray _R1,
			OutputArray _R2, OutputArray _t)
{
	Mat E = _E.getMat().reshape(1, 3);
	CV_Assert(E.cols == 3 && E.rows == 3);

	Mat D, U, Vt;
	SVD::compute(E, D, U, Vt);

	if (determinant(U) < 0)
		U *= -1.;
	if (determinant(Vt) < 0)
		Vt *= -1.;

	Mat W = (Mat_<double>(3, 3) << 0, 1, 0, -1, 0, 0, 0, 0, 1);
	W.convertTo(W, E.type());

	Mat R1, R2, t;
	R1 = U * W * Vt;
	R2 = U * W.t() * Vt;
	t = U.col(2) * 1.0;

	R1.copyTo(_R1);
	R2.copyTo(_R2);
	t.copyTo(_t);
}

