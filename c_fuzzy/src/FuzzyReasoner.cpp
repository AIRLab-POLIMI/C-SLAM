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
	(*inputTable)[name] = value;
}

map<string, double> FuzzyReasoner::run()
{
	//works backwards because MF are stored in reverse order by the parser...
	for (vector<Node*>::reverse_iterator it = knowledgeBase->rbegin();
			it != knowledgeBase->rend(); ++it)
	{
		Node* rule = *it;
		rule->evaluate();
	}

	map<string, DataMap> aggregatedResults = aggregator->getAggregations();

	map<string, double> map;

	return defuzzyfier.defuzzify(aggregatedResults); //FIXME momentaneo
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

FuzzyReasoner::~FuzzyReasoner()
{
	delete inputTable;
	deleteDomains();
	delete domainTable;
	delete aggregator;
	deleteRules();
	delete knowledgeBase;
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
		for(DataMap::iterator j = dataMap.begin(); j != dataMap.end(); ++j)
		{
			Data data = j->second;
			weight += data.weight;
			value += data.weight*data.value;
		}

		if(cardinality > 1)
			value /= weight;

		results[i->first] = value;
	}

	return results;

}
