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

/* Corner detector callback */
void thresholdCorner(int value, void* data)
{
	FeatureDetector* cornerD = static_cast<FeatureDetector*>(data);
	cornerD->setThreshold(value);
}

void windowSizeCorner(int value, void* data)
{
	FeatureDetector* cornerD = static_cast<FeatureDetector*>(data);
	cornerD->setClusterWindow(value);
}

void minClusterSizeCorner(int value, void* data)
{
	FeatureDetector* cornerD = static_cast<FeatureDetector*>(data);
	cornerD->setClusterMinSize(value);
}

void noisebarrierCorner(int value, void* data)
{
	FeatureDetector* cornerD = static_cast<FeatureDetector*>(data);
	cornerD->setNoiseBarrier(value);
}

void objectWindoCorner(int value, void* data)
{
	FeatureDetector* cornerD = static_cast<FeatureDetector*>(data);
	cornerD->setObjectWindow(value);
}

void objectMinSizeCorner(int value, void* data)
{
	FeatureDetector* cornerD = static_cast<FeatureDetector*>(data);
	cornerD->setObjectMinSize(value);
}

void minCanny(int value, void* data)
{
	HoughDetector* borderD = static_cast<HoughDetector*>(data);
	borderD->setThreshold1(value);
}

void maxCanny(int value, void* data)
{
	HoughDetector* borderD = static_cast<HoughDetector*>(data);
	borderD->setThreshold2(value);
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
