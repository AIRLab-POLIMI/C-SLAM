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

#include <boost/make_shared.hpp>

using namespace std;
using namespace boost;

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

	for (DomainTable::iterator it = domain.begin(); it != domain.end(); ++it)
	{
		string domainName = it->first;
		MFTablePtr mfTable = it->second;
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
	for (MFTable::iterator it = newMfTable->begin(); it != newMfTable->end();
				++it)
	{
		const string& label = it->first;

		if (oldMfTable->count(label) == 0)
		{
			MFTable& tableRef = *oldMfTable;
			tableRef[label] = it->second;
		}
	}

}

void FuzzyVariableEngine::buildDomain(vector<string> variables)
{
	DomainTable& domainMap = *domainTable;
	mfTable = make_shared<MFTable>();

	for (vector<string>::iterator it = variables.begin(); it != variables.end();
				it++)
	{
		domainMap[*it] = mfTable;
		Variable variable(currentNamespace, *it);
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
	for (vector<Variable>::iterator it = vars.begin(); it != vars.end(); ++it)
	{

		if (!variableMasks.contains(*it))
		{
			variableMasks.newVariableMask(*it);
		}

		variableMasks.updateVariableMask(*it, rule);
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
