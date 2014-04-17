/*
 * c_fuzzy,
 *
 *
 * Copyright (C) 2014 Davide Tateo
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

#include "FuzzyVariableEngine.h"

using namespace std;

void FuzzyVariableEngine::initializeNamespaces()
{
	namespaceTable = new NamespaceTable();
	domainTable = new DomainTable();
	mfTable = NULL;
	NamespaceTable& namespaceMap = *namespaceTable;
	namespaceMap[""] = domainTable;
}

void FuzzyVariableEngine::initializeMasks()
{
	namespaceMasks = new NamespaceMasks();
	variableMasks = new VariableMasks();
	namespaceMasks->addNameSpace("", variableMasks);
}

void FuzzyVariableEngine::enterNamespace(string nameSpace)
{
	NamespaceTable& namespaceMap = *namespaceTable;

	if (namespaceMap.count(nameSpace) == 0)
	{
		domainTable = new DomainTable();
		namespaceMap[nameSpace] = domainTable;

		variableMasks = new VariableMasks();
		namespaceMasks->addNameSpace(nameSpace, variableMasks);
	}
	else
	{
		domainTable = namespaceMap[nameSpace];

		NamespaceMasks& masksMap = *namespaceMasks;
		variableMasks = masksMap[nameSpace];
	}
}

void FuzzyVariableEngine::addMF(std::string label, FuzzyMF* mf)
{
	MFTable& map = *mfTable;
	map[label] = mf;
}

void FuzzyVariableEngine::buildDomain(std::vector<std::string> variables)
{
	DomainTable& domainMap = *domainTable;
	mfTable = new MFTable();

	for (vector<string>::iterator it = variables.begin(); it != variables.end();
				it++)
	{
		domainMap[*it] = mfTable;
		variableMasks->newVariableMask(*it);
	}
}

NamespaceTable* FuzzyVariableEngine::getTable()
{
	return namespaceTable;
}

NamespaceMasks* FuzzyVariableEngine::getMasks()
{
	return namespaceMasks;
}

VariableMasks& FuzzyVariableEngine::getVariableMasks(std::string nameSpace)
{
	NamespaceMasks& nMask = *namespaceMasks;
	return *nMask[nameSpace];
}

