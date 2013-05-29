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
#include "LineDetector.h"
#include "BorderDetector.h"
#include "CornerDetector.h"

void minCanny(int value, void* data)
{
	BorderDetector* borderD = static_cast<BorderDetector*>(data);
	borderD->setThreshold1(value);
}

void maxCanny(int value, void* data)
{
	BorderDetector* borderD = static_cast<BorderDetector*>(data);
	borderD->setThreshold2(value);
}

void thresholdCorner(int value, void* data)
{
	CornerDetector* cornerD = static_cast<CornerDetector*>(data);
	cornerD->setThreshold(value);
}

void thresholdHoughP(int value, void* data)
{
	ProbabilisticHoughLineDetector* lineD =
			static_cast<ProbabilisticHoughLineDetector*>(data);
	lineD->setThreshold(value);
}

void minLineLengthHoughP(int value, void* data)
{
	ProbabilisticHoughLineDetector* lineD =
			static_cast<ProbabilisticHoughLineDetector*>(data);
	lineD->setMinLineLength(value);
}

void maxLineGapHoughP(int value, void* data)
{
	ProbabilisticHoughLineDetector* lineD =
			static_cast<ProbabilisticHoughLineDetector*>(data);
	lineD->setMaxLineGap(value);
}
