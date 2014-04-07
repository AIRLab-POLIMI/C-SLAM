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

#include "FuzzyKnowledgeBase.h"

using namespace std;

size_t FuzzyKnowledgeBase::size()
{
	return knowledgeBase->size();
}

VariableMasks& FuzzyKnowledgeBase::getVariableMasks()
{
	return *variableMasks;
}

DomainTable& FuzzyKnowledgeBase::getDomaintable()
{
	return *domainTable;
}

Node& FuzzyKnowledgeBase::operator[](const size_t i)
{
	return *knowledgeBase->at(i);
}

void FuzzyKnowledgeBase::addRule(Node* fuzzyRule, vector<string>& variables)
{
	size_t currentRule = knowledgeBase->size();
	for (vector<string>::iterator it = variables.begin(); it != variables.end();
				++it)
	{
		if(!variableMasks->contains(*it))
		{
			variableMasks->newVariableMask(*it);
		}

		variableMasks->updateVariableMask(*it, currentRule);
	}

	knowledgeBase->push_back(fuzzyRule);
	variableMasks->normalizeVariableMasks(knowledgeBase->size());
}



void FuzzyKnowledgeBase::deleteRules()
{
	for (vector<Node*>::iterator it = knowledgeBase->begin();
				it != knowledgeBase->end(); ++it)
	{
		delete *it;
	}
}

void FuzzyKnowledgeBase::deleteMF(MFTable* mfTable)
{
	for (MFTable::iterator it = mfTable->begin(); it != mfTable->end(); ++it)
	{
		delete it->second;
	}
}

void FuzzyKnowledgeBase::deleteDomains()
{
	for (DomainTable::iterator it = domainTable->begin();
				it != domainTable->end(); ++it)
	{
		deleteMF(it->second);
		delete it->second;
	}
}

FuzzyKnowledgeBase::~FuzzyKnowledgeBase()
{
	delete variableMasks;
	deleteDomains();
	delete domainTable;
	deleteRules();
	delete knowledgeBase;
}

