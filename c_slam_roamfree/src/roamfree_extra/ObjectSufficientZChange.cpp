/*
 * c_slam_roamfree,
 *
 *
 * Copyright (C) 2015 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_slam_roamfree.
 *
 * c_slam_roamfree is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_slam_roamfree is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_slam_roamfree.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "roamfree_extra/ObjectSufficientZChange.h"

#include <iostream> //TODO levami

using namespace std;

namespace ROAMvision
{

ObjectSufficientZChange::ObjectSufficientZChange(double minZChange,
			const ObjectObservationMap& zHistory, const double *K) :
			ObjectInitializationStrategy(zHistory, K), _minZChange(minZChange)
{
}

bool ObjectSufficientZChange::initialize()
{
	double zChange = 0;
	if (_zHistory.size() >= 2)
	{

		for (auto i = _zHistory.begin(); i !=  prev(_zHistory.end()); i++)
		{
			for (auto j = next(i); j != _zHistory.end(); j++)
			{
				zChange = std::max(zChange, (i->second.z - j->second.z).norm());
			}
		}

	}

	if (zChange > _minZChange)
	{
		cerr << "sufficient Z change = " << zChange << "min = " << _minZChange << endl;
		return true;
	}

	return false;

}

}
