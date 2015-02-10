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

FuzzyKnowledgeBase::FuzzyKnowledgeBase(FuzzyVariableEngine* variables,
		FuzzyPredicateEngine* predicates, std::vector<NodePtr>* knowledgeBase) :
		variables(variables), predicates(predicates), knowledgeBase(
				knowledgeBase)
{
}

size_t FuzzyKnowledgeBase::size()
{
	return knowledgeBase->size();
}

VariableMasks& FuzzyKnowledgeBase::getMasks()
{
	return variables->getMasks();
}

NamespaceTable& FuzzyKnowledgeBase::getNamespaceTable()
{
	return variables->getTable();
}

Node& FuzzyKnowledgeBase::operator[](const size_t i)
{
	return *knowledgeBase->at(i);
}

void FuzzyKnowledgeBase::addRule(NodePtr fuzzyRule,
		vector<Variable>& vars)
{

	size_t currentRule = knowledgeBase->size();

	variables->updateVariableMask(vars, currentRule);
	knowledgeBase->push_back(fuzzyRule);
	variables->normalizeVariableMasks(knowledgeBase->size());

}

void FuzzyKnowledgeBase::addDomains(string& nameSpace, DomainTable& domain)
{
	variables->addDomains(nameSpace, domain);
}


NodePtr FuzzyKnowledgeBase::getPredicateInstance(string& nameSpace,
		string& predicateName, Variable variable)
{
	PredicateInstance instance = predicates->getPredicateInstance(nameSpace,
			predicateName, variable);
	variables->addDomains(variable.nameSpace, *instance.second);
	this->variables->updateVariableMask(variable, knowledgeBase->size());

	return instance.first;
}

FuzzyKnowledgeBase::~FuzzyKnowledgeBase()
{
	delete variables;
	delete predicates;
	delete knowledgeBase;
}

