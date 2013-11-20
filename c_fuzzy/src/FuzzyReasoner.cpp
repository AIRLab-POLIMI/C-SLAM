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

inline void FuzzyReasoner::addRule(Node* fuzzyRule)
{
	knowledgeBase->push_back(fuzzyRule);
}

void FuzzyReasoner::addInput(string name, int value)
{
	(*inputTable)[name] = value;
	cout << rulesMask[0] << ", " << rulesMask[1] << ", " << rulesMask[3]
				<< endl;
	rulesMask &= *variableMasks->at(name);
	cout << (*variableMasks->at(name))[0] << ", "
			<< (*variableMasks->at(name))[1] << ", "
			<< (*variableMasks->at(name))[3] << endl;
	cout << rulesMask[0] << ", " << rulesMask[1] << ", " << rulesMask[3]
			<< endl;
}

map<string, double> FuzzyReasoner::run()
{
	size_t index = rulesMask.find_first();
	while (index != boost::dynamic_bitset<>::npos)
	{
		Node* rule = knowledgeBase->at(index);
		rule->evaluate();
		index = rulesMask.find_next(index);
	}

	map<string, DataMap> aggregatedResults = aggregator->getAggregations();

	inputTable->clear();
	rulesMask.set();

	return defuzzyfier.defuzzify(aggregatedResults);
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
	for (map<string, boost::dynamic_bitset<>*>::iterator it =
			variableMasks->begin(); it != variableMasks->end(); ++it)
	{
		delete it->second;
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
		int cardinality = dataMap.size();
		for (DataMap::iterator j = dataMap.begin(); j != dataMap.end(); ++j)
		{
			Data data = j->second;
			weight += data.weight;
			value += data.weight * data.value;
		}

		if (cardinality > 1)
			value /= weight;

		results[i->first] = value;
	}

	return results;

}
