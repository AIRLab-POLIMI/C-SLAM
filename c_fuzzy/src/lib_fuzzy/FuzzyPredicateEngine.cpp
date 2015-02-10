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

#include <memory>

using namespace std;

FuzzyPredicateEngine::FuzzyPredicateEngine()
{
	table[""] = make_shared<DomainTable>();
	currentNamespace = "";
	currentTemplateVar = "";
}

void FuzzyPredicateEngine::enterNamespace(string nameSpace)
{
	currentNamespace = nameSpace;

	if (table.count(nameSpace) == 0)
	{
		table[nameSpace] = make_shared<DomainTable>();
	}
}

void FuzzyPredicateEngine::enterPredicate(string templateVariable)
{
	currentTemplateVar = templateVariable;
}

void FuzzyPredicateEngine::buildDomain(std::string templateVar)
{
	DomainTable& domainTable = *table[currentNamespace];

	if (domainTable.count(templateVar) == 0)
	{
		domainTable[templateVar] = make_shared<MFTable>();
	}
	else
	{
		stringstream ss;
		ss << "Error: redefinition of template variable " << templateVar;
		if (!currentNamespace.empty())
			ss << " in class " << currentNamespace;
		throw runtime_error(ss.str());
	}
}

void FuzzyPredicateEngine::addTemplateMF(string label, FuzzyMF* mf)
{
	DomainTable& domainTable = *table[currentNamespace];
	MFTable& mfTable = *domainTable[currentTemplateVar];
	mfTable[label] = FuzzyMFPtr(mf);
}

void FuzzyPredicateEngine::buildPredicate(string name, NodePtr rule)
{
	PredicateData data;
	data.templateVar = currentTemplateVar;
	data.definition = rule;
	predicateMap[currentNamespace][name] = data;
}

PredicateInstance FuzzyPredicateEngine::getPredicateInstance(string nameSpace,
		string predicate, Variable variable)
{
	if (predicateMap.count(nameSpace) == 1
			&& predicateMap[nameSpace].count(predicate) == 1)
	{
		PredicateData data = predicateMap[nameSpace][predicate];
		NodePtr predicate = data.definition->instantiate(variable);
		DomainTablePtr domains = instantiatePredicateVar(nameSpace,
				data.templateVar, variable.domain);
		return PredicateInstance(predicate, domains);
	}

	stringstream ss;
	ss << "Predicate instantiation failed: predicate '";

	if(!nameSpace.empty())
	{
		ss << nameSpace << ".";
	}

	ss << predicate << "' doesn't exists";
	throw runtime_error(ss.str());
}

PredicateInstance FuzzyPredicateEngine::getPredicateInstance(string predicate,
		Variable variable)
{
	return getPredicateInstance("", predicate, variable);
}

DomainTablePtr FuzzyPredicateEngine::instantiatePredicateVar(string nameSpace,
		string templateVar, string variable)
{
	DomainTable& templateDomain = *table[nameSpace];
	MFTablePtr mfTable = templateDomain[templateVar];
	DomainTablePtr domain = make_shared<DomainTable>();
	DomainTable& domainMap = *domain;
	domainMap[variable] = mfTable;

	return domain;
}
