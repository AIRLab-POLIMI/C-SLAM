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

inline void FuzzyReasoner::addRule(Node* fuzzyRule)
{
	knowledgeBase->push_back(fuzzyRule);
}

void FuzzyReasoner::addInput(string name, int value)
{
	if (variableMasks->count(name) != 0)
	{
		(*inputTable)[name] = value;
		inputMask.set(variableMasks->at(name).index, true);
	}
}

map<string, double> FuzzyReasoner::run()
{
	//Calculates the rules to be used
	updateRulesMask();

	//Calculate rules outputs
	size_t index = rulesMask.find_first();
	while (index != boost::dynamic_bitset<>::npos)
	{
		Node* rule = knowledgeBase->at(index);
		rule->evaluate();
		index = rulesMask.find_next(index);
	}

	//Use the aggregation operator
	map<string, DataMap> aggregatedResults = aggregator->getAggregations();

	//clean all input functions
	cleanInputData();

	//return defuzzyfied data
	return defuzzyfier.defuzzify(aggregatedResults);
}

void FuzzyReasoner::updateRulesMask()
{
	boost::dynamic_bitset<> noInputMask(knowledgeBase->size());
	noInputMask.reset();
	for (map<string, BitData>::iterator it = variableMasks->begin();
			it != variableMasks->end(); ++it)
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
	inputTable->clear();
	rulesMask.reset();
	inputMask.reset();
}

void FuzzyReasoner::deleteRules()
{
	for (vector<Node*>::iterator it = knowledgeBase->begin();
			it != knowledgeBase->end(); ++it)
	{
		delete *it;
	}
}

void FuzzyReasoner::deleteMF(MFTable* mfTable)
{
	for (MFTable::iterator it = mfTable->begin(); it != mfTable->end(); ++it)
	{
		delete it->second;
	}
}

void FuzzyReasoner::deleteDomains()
{
	for (DomainTable::iterator it = domainTable->begin();
			it != domainTable->end(); ++it)
	{
		deleteMF(it->second);
		delete it->second;
	}
}
void FuzzyReasoner::deleteMasks()
{
	for (map<string, BitData>::iterator it = variableMasks->begin();
			it != variableMasks->end(); ++it)
	{
		delete it->second.bits;
	}
}

FuzzyReasoner::~FuzzyReasoner()
{
	delete inputTable;
	deleteDomains();
	delete domainTable;
	delete aggregator;
	deleteRules();
	delete knowledgeBase;
	deleteMasks();
	delete variableMasks;
}

map<string, double> Defuzzyfier::defuzzify(map<string, DataMap>& aggregatedData)
{
	map<string, double> results;
	for (map<string, DataMap>::iterator i = aggregatedData.begin();
			i != aggregatedData.end(); ++i)
	{
		DataMap dataMap = i->second;
		double value = 0, weight = 0;
		for (DataMap::iterator j = dataMap.begin(); j != dataMap.end(); ++j)
		{
			Data data = j->second;
			weight += data.weight;
			value += data.weight * data.value;
		}

		value /= weight;

		results[i->first] = value;
	}

	return results;

}
