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

#include <cctype>
#include <fstream>
#include <stdexcept>

#include "FuzzyBuilder.h"
#include "FuzzyOperator.h"
#include "FuzzyRule.h"

using namespace std;

FuzzyBuilder::FuzzyBuilder()
{
	parser = NULL;
	scanner = NULL;
	varEngine = NULL;
	predicateEngine = NULL;
	ruleList = NULL;
	parsingPredicate = false;
}

FuzzyBuilder::~FuzzyBuilder()
{
	if (scanner != NULL)
		delete (scanner);
	if (parser != NULL)
		delete (parser);
}

void FuzzyBuilder::parse(const char *filename)
{
	//Check input file
	ifstream inputFile(filename);
	if (!inputFile.good())
	{
		throw runtime_error("Bad file to parse");
	}

	//initialize scanner and parser
	scanner = new fz::FuzzyScanner(&inputFile);
	parser = new fz::FuzzyParser(*this, *scanner);

	//initialize knowledge base data
	varEngine = new FuzzyVariableEngine();
	predicateEngine = new FuzzyPredicateEngine();
	ruleList = new std::vector<Node*>();
	parsingPredicate = false;

	//parse file
	if (parser->parse() == -1)
	{
		throw runtime_error("Parse Failed");
	}
}

FuzzyKnowledgeBase* FuzzyBuilder::createKnowledgeBase()
{
	varEngine->normalizeVariableMasks(ruleList->size());
	return new FuzzyKnowledgeBase(varEngine, predicateEngine, ruleList);
}

void FuzzyBuilder::buildRule(Node* antecedent, Node* conseguent)
{
	Node* rule = new FuzzyRule(antecedent, conseguent);
	ruleList->push_back(rule);
}

//Predicates
void FuzzyBuilder::enterPredicate(std::string templateVar)
{
	predicateEngine->enterPredicate(templateVar);
	parsingPredicate = true;
}

void FuzzyBuilder::exitPredicate()
{
	parsingPredicate = false;
}

void FuzzyBuilder::buildPredicate(std::string predicateName, Node* definition)
{
	predicateEngine->buildPredicate(predicateName, definition);
}

//Fuzzy operators
Node* FuzzyBuilder::buildAnd(Node* left, Node* right)
{
	return new FuzzyAnd(left, right);
}

Node* FuzzyBuilder::buildOr(Node* left, Node* right)
{
	return new FuzzyOr(left, right);
}

Node* FuzzyBuilder::buildNot(Node* operand)
{
	return new FuzzyNot(operand);
}

Node* FuzzyBuilder::buildIs(pair<string, string> classMember, string mfLabel)
{
	string nameSpace = classMember.first;
	string domain = classMember.second;
	varEngine->updateVariableMask(classMember, ruleList->size());
	return new FuzzyIs(varEngine->getTable(), nameSpace, domain, mfLabel);
}

Node* FuzzyBuilder::buildTemplateIs(string domain, string mfLabel)
{
	return new FuzzyTemplateIs(varEngine->getTable(), "", domain, mfLabel);
}

Node* FuzzyBuilder::buildAssignment(pair<string, string> classMember,
		string label)
{
	string nameSpace = classMember.first;
	string output = classMember.second;
	return new FuzzyAssignment(varEngine->getTable(), nameSpace, output, label);
}

//fuzzy predicates
Node* FuzzyBuilder::getPredicateInstance(string nameSpace, string predicateName,
		pair<string, string> variable)
{
	PredicateInstance instance = predicateEngine->getPredicateInstance(
			nameSpace, predicateName, variable);
	varEngine->addDomains(variable.first, instance.second);
	varEngine->updateVariableMask(variable, ruleList->size());

	return instance.first;
}

Node* FuzzyBuilder::getPredicateInstance(string predicateName,
		pair<string, string> variable)
{
	return getPredicateInstance("", predicateName, variable);
}

//Function to enter a namespace
void FuzzyBuilder::setNameSpace(string nameSpace)
{
	predicateEngine->enterNamespace(nameSpace);
	varEngine->enterNamespace(nameSpace);
}

//Function to return to the default namespace
void FuzzyBuilder::setDefaultNameSpace()
{
	setNameSpace("");
}

//Fuzzy domain
void FuzzyBuilder::buildDomain(string variable)
{
	if (parsingPredicate)
	{
		predicateEngine->buildDomain(variable);
	}
	else
	{
		vector<string> variableVector;
		variableVector.push_back(variable);
		buildDomain(variableVector);
	}
}

void FuzzyBuilder::buildDomain(vector<string> variables)
{
	varEngine->buildDomain(variables);
}

//Fuzzy MF
void FuzzyBuilder::buildMF(string name, string shape, vector<int>& parameters)
{
	FuzzyMF* mf = FuzzyMFEngine::buildMF(name, shape, parameters);

	if (parsingPredicate)
		predicateEngine->addTemplateMF(name, mf);
	else
		varEngine->addMF(name, mf);
}

