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
#include "FuzzyMF.h"
#include "FuzzyRule.h"
#include "FuzzyKnowledgeBase.h"
#include "VariableMasks.h"

#include "FuzzyParser.tab.h"
#include "FuzzyScanner.h"

enum FuzzySets
{
	TOL, TOR, TRA, TRI, INT, SGT
};

class FuzzyBuilder
{

public:
	FuzzyBuilder() :
				parser(NULL), scanner(NULL)
	{
		createMap();
		initializeNameSpaces();

		ruleList = new std::vector<Node*>();
	}

	FuzzyKnowledgeBase* createKnowledgeBase();

	void parse(const char *filename);

	std::ostream& print(std::ostream &stream);

	virtual ~FuzzyBuilder();

public:

	//Function to add a rule to the rulebase
	void buildRule(Node* antecedent, Node* Conseguent);

	//Function to add predicates to the rulebase
	void buildPredicate(std::string predicateName, Node* definition);

	//Functions to build fuzzy operators
	Node* buildAnd(Node* left, Node* right);
	Node* buildOr(Node* left, Node* right);
	Node* buildNot(Node* operand);
	Node* buildAssignment(std::string output, std::string label);
	Node* buildAssignment(std::pair<std::string, std::string> classMember,
				std::string label);
	Node* buildIs(std::string domain, std::string mfLabel);
	Node* buildIs(std::pair<std::string, std::string> classMember,
				std::string mfLabel);
	Node* buildTemplateIs(std::string templateDomain, std::string mfLabel);

	//Functions to manage namespaces
	void setNameSpace(std::string nameSpace);
	void setDefaultNameSpace();

	//Function to build a fuzzy Domain
	void buildDomain(std::string variable);
	void buildDomain(std::vector<std::string> variables);

	//Functions to build fuzzy MF
	void buildMF(std::string name, std::string shape,
				std::vector<int>& parameters);
	FuzzyMF* buildTor(int bottom, int top);
	FuzzyMF* buildTol(int top, int bottom);
	FuzzyMF* buildTra(int bottomLeft, int topLeft, int topRight,
				int bottomRight);
	FuzzyMF* buildTri(int left, int center, int right);
	FuzzyMF* buildInt(int left, int right);
	FuzzyMF* buildSgt(int value);

private:
	void createMap();
	void initializeNameSpaces();
	void checkParameters(std::string name, std::vector<int>& parameters,
				FuzzySets fuzzySetType);
	void chekParametersNumber(std::string name, FuzzySets fuzzySetType,
				size_t parametersSize);
	VariableMasks& getVariableMasks(std::string nameSpace);


private:
	//Data needed to get Builder working
	fz::FuzzyParser* parser;
	fz::FuzzyScanner* scanner;
	std::map<std::string, FuzzySets> fuzzyMap;

	//Data needed to get FuzzyReasoner working
	NamespaceTable* namespaceTable;
	NamespaceMasks* namespaceMasks;
	std::vector<Node*>* ruleList;

	//Data used by the parser for simplicity
	DomainTable* domainTable;
	MFTable* mfTable;
	VariableMasks* variableMasks;

};

#endif /* FUZZYBUILDER_H_ */
