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

#include "FuzzyReasoner.h"

using namespace std;

map<string, FuzzyOutput> Defuzzyfier::defuzzify(
		map<string, DataMap>& aggregatedData)
{
	map<string, FuzzyOutput> results;
	for (map<string, DataMap>::iterator i = aggregatedData.begin();
			i != aggregatedData.end(); ++i)
	{
		DataMap dataMap = i->second;
		double product = 0, weight = 0, value = 0;
		for (DataMap::iterator j = dataMap.begin(); j != dataMap.end(); ++j)
		{
			FuzzyData& data = j->second;
			weight += data.weight;
			value += data.value;
			product += data.weight * data.value;
		}

		FuzzyOutput result;

		if (aggregatedData.size() > 1)
		{
			result.truth = product / value;
			result.value = product / weight;
		}
		else
		{
			result.truth = weight;
			result.value = value;
		}

		results[i->first] = result;
	}

	return results;

}

void FuzzyReasoner::addInput(string name, int value)
{
	inputs[name] = value;
	if (variableMasks.count(name) != 0)
	{
		inputs[name] = value;
		inputMask.set(variableMasks[name].index, true);
	}
}

map<string, FuzzyOutput> FuzzyReasoner::run()
{
	ReasoningData reasoningData(inputs, aggregator);

	//Calculates the rules to be used
	updateRulesMask();

	//Calculate rules outputs
	size_t index = rulesMask.find_first();
	while (index != boost::dynamic_bitset<>::npos)
	{
		Node& rule = knowledgeBase[index];
		rule.evaluate(reasoningData);
		index = rulesMask.find_next(index);
	}

	//Use the aggregation operator
	map<string, DataMap> aggregatedResults = aggregator.getAggregations();

	//clean all input functions
	cleanInputData();

	//return defuzzyfied data
	return defuzzyfier.defuzzify(aggregatedResults);
}

void FuzzyReasoner::updateRulesMask()
{
	boost::dynamic_bitset<> noInputMask(knowledgeBase.size());
	noInputMask.reset();
	for (map<string, BitData>::iterator it = variableMasks.begin();
			it != variableMasks.end(); ++it)
	{
		int index = it->second.index;
		boost::dynamic_bitset<>& currentMask = *it->second.bits;
		if (inputMask[index])
			rulesMask |= currentMask;
		else
			noInputMask |= currentMask;
	}

	rulesMask &= noInputMask.flip();

}

void FuzzyReasoner::cleanInputData()
{
	rulesMask.reset();
	inputMask.reset();
}

