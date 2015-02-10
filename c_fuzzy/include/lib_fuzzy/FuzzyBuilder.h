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

#ifndef FUZZYBUILDER_H_
#define FUZZYBUILDER_H_

#include <string>
#include <vector>
#include <map>

#include "Node.h"
#include "FuzzyKnowledgeBase.h"

#include "FuzzyVariableEngine.h"
#include "FuzzyPredicateEngine.h"
#include "FuzzyMFEngine.h"

class FuzzyBuilder
{

public:
	FuzzyBuilder();

	FuzzyKnowledgeBase* createKnowledgeBase();

	void parse(const char *filename);

public:

	//Function to add a rule to the rulebase
	void buildRule(NodePtr antecedent, NodePtr Conseguent);

	//Function to add predicates to the rulebase
	void enterPredicate(std::string templateVar);
	void exitPredicate();
	void buildPredicate(std::string predicateName, NodePtr definition);

	//Functions to manage namespaces
	void setNameSpace(std::string nameSpace);
	void setDefaultNameSpace();

	//Function to build a fuzzy Domain
	void buildDomain(std::string variable);
	void buildDomain(std::vector<std::string> variables);

	//Functions to build fuzzy operators
	NodePtr buildAnd(NodePtr left, NodePtr right);
	NodePtr buildOr(NodePtr left, NodePtr right);
	NodePtr buildNot(NodePtr operand);
	NodePtr buildAssignment(Variable variable, std::string label);
	NodePtr buildIs(Variable variable, std::string mfLabel);
	NodePtr buildTemplateIs(std::string templateDomain, std::string mfLabel);

	//Functions to get predicate instances
	NodePtr getPredicateInstance(std::string nameSpace, std::string predicateName,
			Variable variable);
	NodePtr getPredicateInstance(std::string predicateName, Variable variable);

	//Function to build fuzzy MF
	void buildMF(std::string name, std::string shape,
			std::vector<int>& parameters);

private:

	//Data needed to build the knowledgeBase
	FuzzyVariableEngine* varEngine;
	FuzzyPredicateEngine* predicateEngine;
	std::vector<NodePtr>* ruleList;

	//Parser state
	bool parsingPredicate;

};

#endif /* FUZZYBUILDER_H_ */
