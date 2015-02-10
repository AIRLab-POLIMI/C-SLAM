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

#include <stdexcept>
#include <sstream>

#include <memory>

using namespace std;

FuzzyVariableEngine::FuzzyVariableEngine()
{
	initializeNamespaces();
}

void FuzzyVariableEngine::enterNamespace(string& nameSpace)
{
	if (namespaceTable.count(nameSpace) == 0)
	{
		domainTable = make_shared<DomainTable>();
		namespaceTable[nameSpace] = domainTable;
	}
	else
	{
		domainTable = namespaceTable[nameSpace];
	}

	currentNamespace = nameSpace;
}

void FuzzyVariableEngine::addMF(string& label, FuzzyMF* mf)
{
	MFTable& map = *mfTable;
	map[label] = FuzzyMFPtr(mf);
}

void FuzzyVariableEngine::checkNameSpaceExistence(string& nameSpace)
{
	if (namespaceTable.count(nameSpace) == 0)
	{
		stringstream ss;
		ss << "Error: non existing class " << nameSpace << endl;
		ss << "You must define a namespace in the knowledgeBase for each class";
		throw runtime_error(ss.str());
	}
}

void FuzzyVariableEngine::addDomains(string& nameSpace, DomainTable& domain)
{
	checkNameSpaceExistence(nameSpace);

	for (auto& it : domain)
	{
		string domainName = it.first;
		MFTablePtr mfTable = it.second;
		DomainTable& domainTable = *namespaceTable[nameSpace];
		if (domainTable.count(domainName) == 0)
		{
			domainTable[domainName] = mfTable;
			Variable variable(nameSpace, domainName);
			variableMasks.newVariableMask(variable);
		}
		else
		{
			joinDomains(domainTable[domainName], mfTable, nameSpace,
						domainName);
		}
	}

}

void FuzzyVariableEngine::joinDomains(MFTablePtr oldMfTable,
			MFTablePtr newMfTable, string& nameSpace, string& domainName)
{
	for (auto& it : *newMfTable)
	{
		const string& label = it.first;

		if (oldMfTable->count(label) == 0)
		{
			MFTable& tableRef = *oldMfTable;
			tableRef[label] = it.second;
		}
	}

}

void FuzzyVariableEngine::buildDomain(vector<string> variables)
{
	DomainTable& domainMap = *domainTable;
	mfTable = make_shared<MFTable>();

	for (auto& var : variables)
	{
		domainMap[var] = mfTable;
		Variable variable(currentNamespace, var);
		variableMasks.newVariableMask(variable);
	}
}
void FuzzyVariableEngine::normalizeVariableMasks(size_t size)
{
	variableMasks.normalizeVariableMasks(size);
}

void FuzzyVariableEngine::updateVariableMask(Variable& var, size_t rule)
{
	variableMasks.updateVariableMask(var, rule);
}

void FuzzyVariableEngine::updateVariableMask(vector<Variable>& vars,
			size_t rule)
{
	for (auto& var : vars)
	{

		if (!variableMasks.contains(var))
		{
			variableMasks.newVariableMask(var);
		}

		variableMasks.updateVariableMask(var, rule);
	}
}

NamespaceTable& FuzzyVariableEngine::getTable()
{
	return namespaceTable;
}

VariableMasks& FuzzyVariableEngine::getMasks()
{
	return variableMasks;
}

void FuzzyVariableEngine::initializeNamespaces()
{
	currentNamespace = "";
	domainTable = make_shared<DomainTable>();
	mfTable.reset();
	namespaceTable[currentNamespace] = domainTable;
}
