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

#include "RobotPose.h"

using namespace cv;

RobotPose::RobotPose()
{
	known = false;

	Rc = Mat::eye(3, 3, CV_64F);
	tc = Mat::zeros(3, 1, CV_64F);
}

void RobotPose::computeCameraMatrices(Mat& P0, Mat& P1, const cv::Mat& R,
			const cv::Mat& t)
{
	writeCameraMatrix(P0, Rc, tc);
	writeCameraMatrix(P1, R * Rc, t + tc);
}

void RobotPose::writeCameraMatrix(Mat& P, const Mat& R, const Mat& t)
{
	P = Mat(3, 4, R.type());
	P(Range::all(), Range(0, 3)) = R;
	P.col(3) = t;
}
