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

#ifndef INCLUDE_PERFORMANCES_PERFORMANCEESTIMATOR_H_
#define INCLUDE_PERFORMANCES_PERFORMANCEESTIMATOR_H_

#include <string>

class PerformanceEstimator
{
public:
	PerformanceEstimator(std::string& objectClass);
	void processNewFrame();
	void tryFeature(std::string& temptativeClass);

	inline double getRecall()
	{
		return static_cast<double>(hit) / static_cast<double>(total);
	}

	inline std::string getObjectClass()
	{
		return objectClass;
	}

	int getHit() const
	{
		return hit;
	}

	int getTotal() const
	{
		return total;
	}

private:
	int hit;
	int total;
	bool wasHit;

	std::string objectClass;

};

extern PerformanceEstimator* pe;

#endif /* INCLUDE_PERFORMANCES_PERFORMANCEESTIMATOR_H_ */
