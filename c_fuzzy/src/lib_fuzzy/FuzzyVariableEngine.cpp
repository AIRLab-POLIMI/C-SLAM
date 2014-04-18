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
	currentNamespace = "";
	namespaceTable = new NamespaceTable();
	domainTable = new DomainTable();
	mfTable = NULL;
	NamespaceTable& namespaceMap = *namespaceTable;
	namespaceMap[currentNamespace] = domainTable;
}

void FuzzyVariableEngine::enterNamespace(string nameSpace)
{
	NamespaceTable& namespaceMap = *namespaceTable;

	if (namespaceMap.count(nameSpace) == 0)
	{
		domainTable = new DomainTable();
		namespaceMap[nameSpace] = domainTable;
	}
	else
	{
		domainTable = namespaceMap[nameSpace];
	}

	currentNamespace = nameSpace;
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
		pair<string, string> variable(currentNamespace, *it);
		namespaceMasks->newVariableMask(variable);
	}
}

void FuzzyVariableEngine::updateVariableMask(std::pair<std::string, std::string>& variable,
				size_t currentRule)
{
	namespaceMasks->updateVariableMask(variable, currentRule);
}

NamespaceTable* FuzzyVariableEngine::getTable()
{
	return namespaceTable;
}

NamespaceMasks* FuzzyVariableEngine::getMasks()
{
	return namespaceMasks;
}

