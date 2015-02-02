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

#include "ParameterServer.h"

#include <angles/angles.h>

ParameterServer::ParameterServer()
{
	dynamic_reconfigure::Server<c_vision::ParametersConfig>::CallbackType f;
	f = boost::bind(&ParameterServer::update, this, _1, _2);
	server.setCallback(f);
}

void ParameterServer::update(c_vision::ParametersConfig &config,
			uint32_t level)
{
	//Setup canny parameters
	canny.automatic = config.canny_automatic;
	canny.alpha = config.canny_alpha;
	canny.apertureSize = config.canny_apertureSize;
	canny.low = config.canny_low;
	canny.high = config.canny_high;
	canny.blur = config.canny_blur;

	//Setup Hough parameters
	hough.rho = config.hough_rho;
	hough.teta = angles::from_degrees(config.hough_teta);
	hough.threshold = config.hough_threshold;
	hough.minLineLenght = config.hough_minLineLenght;
	hough.maxLineGap = config.hough_maxLineGap;

	//Set linefilter parameters
	lineFilter.maxDeltaHorizontal = config.filter_maxDeltaHorizontal;
	lineFilter.maxDeltaVertical = config.filter_maxDeltaVertical;

	//set quadDetector parameters
	quadDetector.verticalOverlap = config.quad_verticalOverlap;
	quadDetector.horizontalOverlap = config.quad_horizontalOverlap;
	quadDetector.polesFormFactor = config.quad_polesFormFactor;
	quadDetector.segmentSupport = config.quad_segmentSupport;

	//Setup clustering parameters
	cluster.threshold = config.cluster_threshold;
	cluster.minPoints = config.cluster_minPoints;
	cluster.maxDistance = config.cluster_maxDistance;

	//Setup classifier parameters
	classifier.threshold = config.classifier_threshold;

}


