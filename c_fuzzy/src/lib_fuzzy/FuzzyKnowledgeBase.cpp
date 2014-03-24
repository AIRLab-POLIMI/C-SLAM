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

void FuzzyKnowledgeBase::addRule(Node* fuzzyRule)
{
	knowledgeBase->push_back(fuzzyRule);
}

size_t FuzzyKnowledgeBase::size()
{
	return knowledgeBase->size();
}

map<string, BitData>& FuzzyKnowledgeBase::getVariableMasks()
{
	return *variableMasks;
}

void FuzzyKnowledgeBase::deleteRules()
{
	for (vector<Node*>::iterator it = knowledgeBase->begin();
			it != knowledgeBase->end(); ++it)
	{
		delete *it;
	}
}

void FuzzyKnowledgeBase::deleteMasks()
{
	for (map<string, BitData>::iterator it = variableMasks->begin();
			it != variableMasks->end(); ++it)
	{
		delete it->second.bits;
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

Node& FuzzyKnowledgeBase::operator[](const unsigned long i)
{
	return *knowledgeBase->at(i);
}

FuzzyKnowledgeBase::~FuzzyKnowledgeBase()
{
	deleteMasks();
	delete variableMasks;
	deleteDomains();
	delete domainTable;
	deleteRules();
	delete knowledgeBase;
}

