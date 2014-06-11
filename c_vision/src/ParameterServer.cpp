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

#include "DefaultParameters.h"

ParameterServer::ParameterServer(ros::NodeHandle& n) :
			n(n)
{
	getCanny();
}

CannyParam& ParameterServer::getCannyParams()
{
	return canny;
}

HougParam& ParameterServer::getHoughParams()
{
	return hough;
}

DBScanParam& ParameterServer::getDBScanParams()
{
	return dbscan;
}

FeatureParam& ParameterServer::getFeatureParams()
{
	return features;
}

LineParam& ParameterServer::getLineParams()
{
	return lines;
}

void ParameterServer::getCanny()
{
	if(n.getParamCached("canny/alpha", canny.alpha))
		ROS_INFO("canny alpha parameter setted");
	else
		ROS_WARN("canny alpha parameter not setted");
}

