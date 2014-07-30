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

void RobotPose::addObjectPose(Mat& R, Mat& t)
{
	rotationList.push_back(R);
	translationList.push_back(t);
}

void RobotPose::updateRobotPose(std::string camera_frame)
{

	size_t size = rotationList.size();

	if (size > 0)
	{
		tc = translationList[0];
		for (int i = 1; i < size; i++)
		{
			tc += translationList[i];
		}

		tc *= 1.0 / size;

		//TODO cambiare
		Rc = rotationList[0];

		//clear pose lists
		rotationList.clear();
		translationList.clear();
	}

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(tc(0, 0), tc(1, 0), tc(2, 0)));
	tf::Matrix3x3 Rtf( //
				Rc(0, 0), Rc(0, 1), Rc(0, 2), //
				Rc(1, 0), Rc(1, 1), Rc(1, 2), //
				Rc(2, 0), Rc(2, 1), Rc(2, 2));
	transform.setBasis(Rtf);

	br.sendTransform(
				tf::StampedTransform(transform, ros::Time::now(), "map",
							camera_frame));
}
