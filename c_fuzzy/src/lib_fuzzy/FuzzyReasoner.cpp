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
#include <iostream>

using namespace std;

OutputTable Defuzzyfier::defuzzify(AggregationMap& aggregatedData)
{
	OutputTable results;

	for (auto& k : aggregatedData)
	{
		string nameSpace = k.first;
		DomainAggregationMap& domainMap = k.second;

		for (auto& i : domainMap)
		{
			string output = i.first;
			DataMap& dataMap = i.second;
			double product = 0, weight = 0, value = 0;
			for (auto& j : dataMap)
			{
				FuzzyData& data = j.second;
				weight += data.weight;
				value += data.value;
				product += data.weight * data.value;
			}

			FuzzyOutput result;

			if (dataMap.size() > 1)
			{
				result.truth = product / value;
				result.value = product / weight;
			}
			else
			{
				result.truth = weight;
				result.value = value;
			}

			results[nameSpace][output] = result;
		}
	}

	return results;

}

FuzzyReasoner::FuzzyReasoner(FuzzyKnowledgeBase& knowledgeBase) :
			knowledgeBase(knowledgeBase),
			variableMasks(knowledgeBase.getMasks())
{
	rulesMask.resize(knowledgeBase.size(), false);
	inputMask.resize(variableMasks.size(), false);

	rulesMask.reset();
	inputMask.reset();
}

void FuzzyReasoner::addInput(Variable variable, int value)
{
	if (variableMasks.contains(variable))
	{
		size_t index = variableMasks.getMaskIndex(variable);
		inputs[variable.nameSpace][variable.domain] = value;
		inputMask.set(index, true);
	}
}

void FuzzyReasoner::addInput(string nameSpace, InputMembers& members)
{
	for(auto& it : members)
	{
		Variable input(nameSpace, it.first);
		addInput(input, it.second);
	}
}

void FuzzyReasoner::addInput(string nameSpace, string name, int value)
{
	Variable variable(nameSpace, name);
	addInput(variable, value);
}

void FuzzyReasoner::addInput(string name, int value)
{
	addInput("", name, value);
}

OutputTable FuzzyReasoner::run()
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
	AggregationMap aggregatedResults = aggregator.getAggregations();

	//clean all input functions
	cleanInputData();

	//return defuzzyfied data
	return defuzzyfier.defuzzify(aggregatedResults);
}

void FuzzyReasoner::updateRulesMask()
{
	boost::dynamic_bitset<> noInputMask(knowledgeBase.size());
	noInputMask.reset();

	for (size_t index = 0; index < variableMasks.size(); index++)
	{
		boost::dynamic_bitset<>& currentMask = variableMasks[index];
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

