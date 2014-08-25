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
		//TODO consider multiple targets
		tc = translationList[0];
		Rc = rotationList[0];

		//send updated pose
		sendTransform(camera_frame);

		//clear pose lists
		rotationList.clear();
		translationList.clear();

		//pose known
		known = true;
	}
	else
	{
		//pose unknown
		known = false;
	}

}
void RobotPose::writeCameraMatrix(Mat& P, const Mat& R, const Mat& t)
{
	P = Mat(3, 4, R.type());
	P(Range::all(), Range(0, 3)) = R;
	P.col(3) = t;
}

void RobotPose::sendTransform(const std::string& camera_frame)
{
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

