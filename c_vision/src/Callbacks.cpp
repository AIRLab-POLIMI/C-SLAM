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

#include "Callbacks.h"
#include "FeatureDetector.h"
#include "HoughDetector.h"
#include "DBScan.h"

/* Corner detector callback */
void thresholdCorner(int value, void* data)
{
	FeatureDetector* cornerD = static_cast<FeatureDetector*>(data);
	cornerD->setThreshold(value);
}

void maxDistanceCluster(int maxDistance, void* data)
{
	DBScan* clusterD = static_cast<DBScan*>(data);
	clusterD->setMaxDistance(maxDistance);
}

void minPointsCluster(int minPoints, void* data)
{
	DBScan* clusterD = static_cast<DBScan*>(data);
	clusterD->setMinPoints(minPoints);
}

void thresholdHoughP(int value, void* data)
{
	HoughDetector* lineD = static_cast<HoughDetector*>(data);
	lineD->setThreshold(value);
}

void minLineLengthHoughP(int value, void* data)
{
	HoughDetector* lineD = static_cast<HoughDetector*>(data);
	lineD->setMinLineLength(value);
}

void maxLineGapHoughP(int value, void* data)
{
	HoughDetector* lineD = static_cast<HoughDetector*>(data);
	lineD->setMaxLineGap(value);
}
