/*
 * c_vision,
 *
 *
 * Copyright (C) 2015 Davide Tateo
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

#include "performances/PerformanceEstimator.h"

PerformanceEstimator::PerformanceEstimator(std::string& objectClass) :
			objectClass(objectClass)
{
	wasHit = false;
	hit = 0;
	total = 0;
}

void PerformanceEstimator::processNewFrame()
{
	total++;
	wasHit = false;
}

void PerformanceEstimator::tryFeature(std::string& temptativeClass)
{
	if(!wasHit && temptativeClass == objectClass)
	{
		hit++;
		wasHit = true;
	}
}

std::string classname = "PossibleDoor";
PerformanceEstimator* pe = new PerformanceEstimator(classname);

