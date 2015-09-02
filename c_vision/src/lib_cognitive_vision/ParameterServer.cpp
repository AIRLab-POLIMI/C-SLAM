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
#include <ros/ros.h>
#include <vector>

using namespace std;

ParameterServer::ParameterServer()
{
	dynamic_reconfigure::Server<c_vision::ParametersConfig>::CallbackType f;
	f = boost::bind(&ParameterServer::update, this, _1, _2);
	server.setCallback(f);

	vector<double> K_std(9);
	if (!ros::param::get("K", K_std) || K_std.size() != 9)
	{
		throw runtime_error(
					"Incorrect initial pose or no initial pose specified");
	}

	if (!ros::param::get("camera_source", camera_source))
	{
		throw runtime_error("No camera source specified");
	}

	if (!ros::param::get("imu_source", imu_source))
	{
		throw runtime_error("No imu source specified");
	}

	quadDetector.K;
	quadDetector.K << K_std[0], K_std[1], K_std[2],
	/*              */K_std[3], K_std[4], K_std[5],
	/*              */K_std[6], K_std[7], K_std[8];

	quadDetector.omega = quadDetector.K.transpose().inverse()
				* quadDetector.K.inverse();
}

void ParameterServer::update(c_vision::ParametersConfig &config, uint32_t level)
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
	quadDetector.points = config.quad_points;
	quadDetector.threshold = config.quad_threshold;
	quadDetector.maxDistance = config.quad_maxDistance;
	quadDetector.minPoints = config.quad_minPoints;

	//set up corner classifier params
	cornerClass.kernelSize = config.corner_kernelSize;
	cornerClass.bucketWidth = config.corner_bucketWidth;

	//Setup clustering parameters
	cluster.threshold = config.cluster_threshold;
	cluster.minPoints = config.cluster_minPoints;
	cluster.maxDistance = config.cluster_maxDistance;

	//Setup classifier parameters
	classifier.threshold = config.classifier_threshold;

	//Setup display params
	display.currentObject = config.displayer_currentObject;

}

