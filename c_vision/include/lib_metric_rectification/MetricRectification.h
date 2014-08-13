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

cv::Mat metricRectify(cv::Mat& K, cv::Vec3f& v1, cv::Vec3f& v2);

void findConicDualCircularPoints(const cv::Mat& W, const cv::Vec3f& linf,
			cv::Mat& Cinf);

cv::Mat findHomography(const cv::Mat& Cinf);

void intersectConicLine(const cv::Mat& C, const cv::Vec3f& l,
			std::vector<std::complex<float> >& I,
			std::vector<std::complex<float> >& J);

void getPointsOnLine(cv::Vec3f l, cv::Vec3f p1, cv::Vec3f p2);

cv::Mat scalarProduct(const std::vector<std::complex<float> >& I,
			const std::vector<std::complex<float> >& J);

}

#endif /* METRICRECTIFICATION_H_ */
