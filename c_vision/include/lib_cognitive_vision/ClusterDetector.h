/*
 * c_vision,
 *
 *
 * Copyright (C) 2013 Davide Tateo
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

#ifndef FEATUREDETECTOR_H_
#define FEATUREDETECTOR_H_

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>


#include "DBSCAN.h"
#include "ParameterServer.h"

class ClusterDetector
{
public:
	ClusterDetector(ClusterParam& params);
	std::vector<Cluster>* detect(cv::Mat& input, const cv::Mat& mask = cv::Mat());

private:
	ClusterParam& params;
	DBSCAN clustering;

};

#endif /* FEATUREDETECTOR_H_ */
