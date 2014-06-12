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

ParameterServer::ParameterServer(ros::NodeHandle& n) :
			n(n)
{
	getCanny();
	getHough();
	getCluster();
	getClassifier();

	ros::Timer timer = n.createTimer(ros::Duration(1),
				&ParameterServer::updateParameters, this);
}

void ParameterServer::updateParameters(const ros::TimerEvent& event)
{
	//update Canny parameters
	n.getParamCached("canny/alpha", canny.alpha);
	n.getParamCached("canny/apertureSize", canny.apertureSize);

	//update cluster parameters
	n.getParamCached("cluster/threshold", cluster.threshold);
	n.getParamCached("cluster/minPoints", cluster.minPoints);
	n.getParamCached("cluster/maxDistance", cluster.maxDistance);

	//update hough parameters
	double tetaDegrees;
	n.getParamCached("hough/rho", hough.rho);
	n.getParamCached("hough/teta", tetaDegrees);
	hough.teta = angles::from_degrees(tetaDegrees);
	n.getParamCached("hough/threshold", hough.threshold);
	n.getParamCached("hough/minLineLenght", hough.minLineLenght);
	n.getParamCached("hough/maxLineGap", hough.maxLineGap);

	//update classifiers parameters
	n.getParamCached("classifier/threshold", classifier.threshold);
}

void ParameterServer::getCanny()
{
	if (n.getParamCached("canny/alpha", canny.alpha)
				&& n.getParamCached("canny/apertureSize", canny.apertureSize))
		ROS_INFO("canny parameters setted");
	else
		ROS_WARN("canny parameters not setted");
}

void ParameterServer::getHough()
{
	double tetaDegrees;
	if (n.getParamCached("hough/rho", hough.rho)
				&& n.getParamCached("hough/teta", tetaDegrees)
				&& n.getParamCached("hough/threshold", hough.threshold)
				&& n.getParamCached("hough/minLineLenght", hough.minLineLenght)
				&& n.getParamCached("hough/maxLineGap", hough.maxLineGap))
	{
		hough.teta = angles::from_degrees(tetaDegrees);
		ROS_INFO("hough parameters setted");
	}

	else
		ROS_WARN("hough parameters not setted");

}

void ParameterServer::getClassifier()
{
	if (n.getParamCached("classifier/threshold", classifier.threshold))
		ROS_INFO("classifier parameters setted");
	else
		ROS_WARN("classifier parameters not setted");

}

void ParameterServer::getCluster()
{
	if (n.getParamCached("cluster/threshold", cluster.threshold)
				&& n.getParamCached("cluster/minPoints", cluster.minPoints)
				&& n.getParamCached("cluster/maxDistance", cluster.maxDistance))
		ROS_INFO("cluster parameters setted");
	else
		ROS_WARN("cluster parameters not setted");
}
