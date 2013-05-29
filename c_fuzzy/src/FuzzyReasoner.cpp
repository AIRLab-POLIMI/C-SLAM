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

inline void FuzzyReasoner::addRule(Node* fuzzyRule)
{
	knowledgeBase->push_back(fuzzyRule);
}

inline void FuzzyReasoner::addInput(std::string name, int value)
{
	(*inputTable)[name] = value;
}

std::map<std::string, double> FuzzyReasoner::run()
{
	for (std::vector<Node*>::iterator it = knowledgeBase->begin();
				it != knowledgeBase->end(); ++it)
	{
		Node* rule = *it;
		rule->evaluate();
	}

	return aggregator->getOutputs();
}

void FuzzyReasoner::deleteRules()
{
	for (std::vector<Node*>::iterator it = knowledgeBase->begin();
			it != knowledgeBase->end(); ++it)
	{
		delete *it;
	}
}

void FuzzyReasoner::deleteMF()
{
	for (std::map<std::string, FuzzyMF*>::iterator it = mfTable->begin();
			it != mfTable->end(); ++it)
	{
		delete it->second;
	}
}

FuzzyReasoner::~FuzzyReasoner()
{
	delete inputTable;
	deleteMF();
	delete mfTable;
	delete aggregator;
	deleteRules();
	delete knowledgeBase;
}

