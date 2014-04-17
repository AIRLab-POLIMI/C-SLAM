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

#include "FuzzyPredicateEngine.h"

#include <stdexcept>
#include <sstream>

using namespace std;

void FuzzyPredicateEngine::enterNamespace(string nameSpace)
{
	currentNamespace = nameSpace;

	if (table.count(nameSpace) == 0)
	{
		table[nameSpace] = new DomainTable();
	}
}

void FuzzyPredicateEngine::enterPredicate(string templateVariable)
{
	currentTemplateVar = templateVariable;
}

void FuzzyPredicateEngine::buildDomain(std::string templateVar)
{
	DomainTable& domainTable = *table[currentNamespace];

	if(domainTable.count(templateVar) == 0)
	{
		domainTable[templateVar] = new MFTable();
	}
	else
	{
		stringstream ss;
		ss << "error Redefinition of template variable " << templateVar;
		if(!currentNamespace.empty())
			ss << "in class " << currentNamespace;
		throw logic_error(ss.str());
	}
}

void FuzzyPredicateEngine::addTemplateMF(string label, FuzzyMF* mf)
{
	DomainTable& domainTable = *table[currentNamespace];
	MFTable& mfTable = *domainTable[currentTemplateVar];
	mfTable[label] = mf;
}

void FuzzyPredicateEngine::buildPredicate(string name, Node* rule)
{
	PredicateData data;
	data.templateVar = currentTemplateVar;
	data.definition = rule;
	predicateMap[currentNamespace][name] = data;
}

Node* FuzzyPredicateEngine::getPredicateInstance(string nameSpace,
			string predicate, pair<string, string> variable)
{
	if (predicateMap.count(nameSpace) == 1
				&& predicateMap[nameSpace].count(predicate) == 1)
	{
		PredicateData data = predicateMap[nameSpace][predicate];
		return data.definition->instantiate(variable);
	}

	throw logic_error("Predicate instantiation failed: predicate doesn't exists");
}

Node* FuzzyPredicateEngine::getPredicateInstance(string predicate,
			pair<string, string> variable)
{
	return getPredicateInstance("", predicate, variable);
}
