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

#ifndef OPENCV3_H_
#define OPENCV3_H_

#include <opencv2/opencv.hpp>

namespace cv3
{
//Opencv3 function, use original ones when they come out
void decomposeEssentialMat(cv::InputArray _E, cv::OutputArray _R1,
			cv::OutputArray _R2, cv::OutputArray _t);
int recoverPose(cv::InputArray E, cv::InputArray _points1,
			cv::InputArray _points2, cv::OutputArray _R, cv::OutputArray _t,
			double focal = 1.0, cv::Point2d pp = cv::Point2d(0, 0),
			cv::InputOutputArray _mask = cv::noArray());
}

#endif /* OPENCV3_H_ */
