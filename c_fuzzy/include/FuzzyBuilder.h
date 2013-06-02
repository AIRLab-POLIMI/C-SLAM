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

#include<string>
#include<vector>
#include<map>

#include "Node.h"
#include "FuzzyMF.h"
#include "FuzzyRule.h"
#include "FuzzyReasoner.h"
#include "FuzzyScanner.h"
#include "FuzzyParser.tab.h"


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
		inputTable = new std::map<std::string, int>();
		aggregator = new FuzzyAggregator();
		domainTable = new DomainTable();
		mfTable = NULL;
		ruleList = new std::vector<Node*>();
	}

	FuzzyReasoner* createReasoner();

	void parse(const char *filename);

	std::ostream& print(std::ostream &stream);

	virtual ~FuzzyBuilder();

private:
	//Data needed to get Builder working
	yy::FuzzyParser* parser;
	FuzzyScanner* scanner;
	std::map<std::string, FuzzySets> fuzzyMap;

	//Data needed to get FuzzyReasoner working
	std::map<std::string, int>* inputTable;
	FuzzyAggregator* aggregator;
	DomainTable* domainTable;
	MFTable* mfTable;
	std::vector<Node*>* ruleList;

public:

	//Function to add a rule to the rulebase
	void buildRule(Node* antecedent, Node* Conseguent);

	//Functions to build fuzzy operators
	Node* buildAnd(Node* left, Node* right);
	Node* buildOr(Node* left, Node* right);
	Node* buildNot(Node* operand);
	Node* buildAssignment(std::string* output, std::string* label);
	Node* buildIs(Node* crispData, std::string* mfLabel);

	//Function to build a fuzzy Domain
	void buildDomain(std::vector<std::string> variables);

	//Functions to build fuzzy MF
	void buildMF(std::string* name, std::string* shape,
			std::vector<int>* parameters);
	FuzzyMF* buildTor(int bottom, int top);
	FuzzyMF* buildTol(int top, int bottom);
	FuzzyMF* buildTra(int bottomLeft, int topLeft, int topRight,
			int bottomRight);
	FuzzyMF* buildTri(int left, int center, int right);
	FuzzyMF* buildInt(int left, int right);
	FuzzyMF* buildSgt(int value);

	//Functions to build crisp data
	Node* buildCrispData(std::string* label);

private:
	void createMap();

};

#endif /* FUZZYBUILDER_H_ */
