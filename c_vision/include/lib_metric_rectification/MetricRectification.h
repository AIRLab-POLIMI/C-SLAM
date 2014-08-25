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

#ifndef METRICRECTIFICATION_H_
#define METRICRECTIFICATION_H_

#include <opencv2/opencv.hpp>

namespace metric_rectification
{

void findLine(const cv::Point& a, const cv::Point& b, cv::Vec3d& l);

cv::Mat metricRectify(const cv::Mat& K, cv::Vec3d& v1, cv::Vec3d& v2);

cv::Mat getScaleTranslationAndRotation(const cv::Vec3d& origin,
			const cv::Vec3d& vertical, double height);

void findConicDualCircularPoints(const cv::Mat& W, const cv::Vec3d& linf,
			cv::Mat& Cinf);

cv::Mat findHomography(const cv::Mat& Cinf);

void intersectConicLine(const cv::Mat& C, const cv::Vec3d& l,
			std::vector<std::complex<double> >& I,
			std::vector<std::complex<double> >& J);

void getPointsOnLine(const cv::Vec3d& l, cv::Vec3d& p1, cv::Vec3d& p2);

cv::Mat scalarProduct(const std::vector<std::complex<double> >& I,
			const std::vector<std::complex<double> >& J);

}

#endif /* METRICRECTIFICATION_H_ */
