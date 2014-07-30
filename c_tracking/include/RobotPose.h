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

#ifndef ROBOTPOSE_H_
#define ROBOTPOSE_H_

#include <opencv2/opencv.hpp>
#include <tf/transform_broadcaster.h>
#include <string>

class RobotPose
{
public:
	RobotPose();
	void computeCameraMatrices(cv::Mat& P0, cv::Mat& P1, const cv::Mat& R,
				const cv::Mat& t);
	void addObjectPose(cv::Mat& R, cv::Mat& t);

	void updateRobotPose(std::string camera_frame);

private:
	void writeCameraMatrix(cv::Mat& P, const cv::Mat& R, const cv::Mat& t);

private:
	bool known;

	//pose estimated wrt object
	std::vector<cv::Mat> rotationList;
	std::vector<cv::Mat> translationList;

	//Current Pose
	cv::Mat_<float> Rc;
	cv::Mat_<float> tc;

	//Tf broadcaster
	tf::TransformBroadcaster br;

};

#endif /* ROBOTPOSE_H_ */
