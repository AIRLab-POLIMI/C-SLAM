/*
 * c_tracking,
 *
 *
 * Copyright (C) 2015 Davide Tateo
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

#include "ParameterServer.h"

#include <angles/angles.h>

ParameterServer::ParameterServer()
{
	dynamic_reconfigure::Server<c_tracking::ParametersConfig>::CallbackType f;
	f = boost::bind(&ParameterServer::update, this, _1, _2);
	server.setCallback(f);
}

void ParameterServer::update(c_tracking::ParametersConfig &config,
			uint32_t level)
{
	//set the extractor parameters
	extraction.threshold = config.extraction_threshold;

	//Set bounding box parameters
	boundingBox.xScaling = config.boundingBox_xScaling;
	boundingBox.yScaling = config.boundingBox_yScaling;

	//Set the outlier rejection parameters
	outlier.maxAngle = angles::from_degrees(config.outlier_maxAngle);
}
