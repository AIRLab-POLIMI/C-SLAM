/*
 * c_fuzzy,
 *
 *
 * Copyright (C) 2013 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_fuzzy.
 *
 * c_fuzzy is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_fuzzy is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_fuzzy.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "FuzzyAggregator.h"

void FuzzyAggregator::addValue(std::string name, double weight, double value)
{
	if (aggregationMap.count(name) == 0)
	{
		Couple couple;
		couple.sumOfValues = value;
		couple.sumOfWeights = weight;
		aggregationMap[name] = couple;
	}
	else
	{
		Couple couple = aggregationMap[name];
		couple.sumOfWeights += weight;
		couple.sumOfValues += value;
	}
}

std::map<std::string, double> FuzzyAggregator::getOutputs()
{
	std::map<std::string, double> outputMap;
	for (std::map<std::string, Couple>::iterator it = aggregationMap.begin();
			it != aggregationMap.end(); ++it)
	{
		double output = it->second.sumOfValues / it->second.sumOfWeights;
		outputMap[it->first] = output;
	}

	return outputMap;
}
