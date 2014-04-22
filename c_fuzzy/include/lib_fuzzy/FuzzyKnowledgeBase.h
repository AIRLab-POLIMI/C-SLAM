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

#ifndef FUZZYKNOWLEDGEBASE_H_
#define FUZZYKNOWLEDGEBASE_H_

#include <map>
#include <string>
#include <vector>

#include "Node.h"
#include "FuzzyVariableEngine.h"
#include "FuzzyPredicateEngine.h"

class FuzzyKnowledgeBase
{
public:
	FuzzyKnowledgeBase(FuzzyVariableEngine* variables,
			FuzzyPredicateEngine* predicates,
			std::vector<Node*>* knowledgeBase);
	size_t size();
	VariableMasks& getMasks();
	NamespaceTable& getNamespaceTable();
	Node& operator[](size_t i);

	void addRule(Node* fuzzyRule,
			std::vector<std::pair<std::string, std::string> >& vars);

	~FuzzyKnowledgeBase();

private:

	void deleteRules();
private:
	FuzzyVariableEngine* variables;
	FuzzyPredicateEngine* predicates;
	std::vector<Node*>* knowledgeBase;
};

#endif /* FUZZYKNOWLEDGEBASE_H_ */
