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

#include "ReasoningData.h"
#include <iostream>

using namespace std;

void FuzzyAggregator::addValue(string output, string mfLabel, double weight,
			double value)
{
	if (weight == 0)
		return;

	if (aggregationMap.count(output) == 0)
	{
		DataMap dataMap = createDataMap(mfLabel, value, weight);
		aggregationMap[output] = dataMap;
	}
	else
	{
		DataMap& dataMap = aggregationMap[output];
		if (dataMap.count(mfLabel) == 0)
		{
			aggregationMap[output][mfLabel] = createData(value, weight);
		}
		else
		{
			FuzzyData& data = aggregationMap[output][mfLabel];
			data.weight += weight;
			data.cardinality++;
		}
	}
}

map<string, DataMap> FuzzyAggregator::getAggregations()
{
	map<string, DataMap> aggregationsOutput;
	for (map<string, DataMap>::iterator it = aggregationMap.begin();
				it != aggregationMap.end(); ++it)
	{
		aggregationsOutput[it->first] = getAggregation(it->second);

	}

	aggregationMap.clear();

	return aggregationsOutput;
}

FuzzyData FuzzyAggregator::createData(double value, double weight)
{
	FuzzyData data;
	data.value = value;
	data.weight = weight;
	data.cardinality = 1;
	return data;
}

DataMap FuzzyAggregator::createDataMap(string mfLabel, double value,
			double weight)
{
	DataMap dataMap;
	dataMap[mfLabel] = createData(value, weight);
	return dataMap;
}

DataMap FuzzyAggregator::getAggregation(DataMap outputs)
{
	DataMap outputMap;
	for (DataMap::iterator it = outputs.begin(); it != outputs.end(); ++it)
	{
		FuzzyData output;

		output.cardinality = it->second.cardinality;
		output.value = it->second.value;
		output.weight = it->second.weight / output.cardinality;

		outputMap[it->first] = output;
	}

	return outputMap;
}

