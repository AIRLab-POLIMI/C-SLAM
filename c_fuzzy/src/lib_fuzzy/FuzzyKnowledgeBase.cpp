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

NamespaceMasks& FuzzyKnowledgeBase::getNamespaceMasks()
{
	return *namespaceMasks;
}

NamespaceTable& FuzzyKnowledgeBase::getNamespaceTable()
{
	return *namespaceTable;
}

Node& FuzzyKnowledgeBase::operator[](const size_t i)
{
	return *knowledgeBase->at(i);
}

void FuzzyKnowledgeBase::addRule(Node* fuzzyRule,
		vector<pair<string, string> >& variables)
{

	size_t currentRule = knowledgeBase->size();
	for (vector<pair<string, string> >::iterator it = variables.begin();
			it != variables.end(); ++it)
	{
		string nameSpace = it->first;
		string variableName = it->second;
		VariableMasks* variableMasks;

		if (!namespaceMasks->contains(nameSpace))
		{
			variableMasks = new VariableMasks();
			namespaceMasks->addNameSpace(nameSpace, variableMasks);
			variableMasks->newVariableMask(variableName);
		}
		else
		{
			NamespaceMasks& map = *namespaceMasks;
			variableMasks = map[nameSpace];
		}

		if(!variableMasks->contains(variableName))
		{
			variableMasks->newVariableMask(variableName);
		}

		variableMasks->updateVariableMask(variableName, currentRule);
	}

	knowledgeBase->push_back(fuzzyRule);
	namespaceMasks->normalizeVariableMasks(knowledgeBase->size());
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
	for (NamespaceTable::iterator it1 = namespaceTable->begin();
			it1 != namespaceTable->end(); ++it1)
	{
		DomainTable* domain = it1->second;
		for (DomainTable::iterator it2 = domain->begin(); it2 != domain->end();
				++it2)
		{
			deleteMF(it2->second);
			delete it2->second;
		}

		delete domain;

	}

}

FuzzyKnowledgeBase::~FuzzyKnowledgeBase()
{
	delete namespaceMasks;
	deleteDomains();
	delete namespaceTable;
	deleteRules();
	delete knowledgeBase;
}

