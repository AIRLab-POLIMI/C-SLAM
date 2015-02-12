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

#include "FuzzyParser.tab.h"
#include "FuzzyScanner.h"

using namespace std;

FuzzyBuilder::FuzzyBuilder()
{
	varEngine = NULL;
	predicateEngine = NULL;
	ruleList = NULL;
	parsingPredicate = false;
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
	fz::FuzzyScanner* scanner = new fz::FuzzyScanner(&inputFile);
	fz::FuzzyParser* parser = new fz::FuzzyParser(*this, *scanner);

	//initialize knowledge base data
	varEngine = new FuzzyVariableEngine();
	predicateEngine = new FuzzyPredicateEngine();
	ruleList = new std::vector<NodePtr>();
	parsingPredicate = false;

	//parse file
	if (parser->parse() == -1)
	{
		throw runtime_error("Parse Failed");
	}

	if (scanner)
		delete scanner;
	if (parser)
		delete parser;
}

FuzzyKnowledgeBase* FuzzyBuilder::createKnowledgeBase()
{
	varEngine->normalizeVariableMasks(ruleList->size());
	return new FuzzyKnowledgeBase(varEngine, predicateEngine, ruleList);
}

void FuzzyBuilder::buildRule(NodePtr antecedent, NodePtr conseguent)
{
	NodePtr rule = make_shared<FuzzyRule>(antecedent, conseguent);
	ruleList->push_back(rule);
}

//Predicates
void FuzzyBuilder::enterPredicate(std::vector<std::string> templateVarList)
{
	predicateEngine->enterPredicate(templateVarList);
	parsingPredicate = true;
}

void FuzzyBuilder::exitPredicate()
{
	parsingPredicate = false;
}

void FuzzyBuilder::buildPredicate(std::string predicateName, NodePtr definition)
{
	predicateEngine->buildPredicate(predicateName, definition);
}

//Fuzzy operators
NodePtr FuzzyBuilder::buildAnd(NodePtr left, NodePtr right)
{
	return make_shared<FuzzyAnd>(left, right);
}

NodePtr FuzzyBuilder::buildOr(NodePtr left, NodePtr right)
{
	return make_shared<FuzzyOr>(left, right);
}

NodePtr FuzzyBuilder::buildNot(NodePtr operand)
{
	return make_shared<FuzzyNot>(operand);
}

NodePtr FuzzyBuilder::buildIs(Variable classMember, string mfLabel)
{
	string nameSpace = classMember.nameSpace;
	string domain = classMember.domain;
	varEngine->updateVariableMask(classMember, ruleList->size());
	return make_shared<FuzzyIs>(varEngine->getTable(), nameSpace, domain,
				mfLabel);
}

NodePtr FuzzyBuilder::buildTemplateIs(string domain, string mfLabel)
{
	size_t domainIndex = predicateEngine->getTemplateVarIndex(domain);
	return make_shared<FuzzyTemplateIs>(varEngine->getTable(), domainIndex,
				mfLabel);
}

NodePtr FuzzyBuilder::buildAssignment(Variable classMember, string label)
{
	string nameSpace = classMember.nameSpace;
	string output = classMember.domain;
	return make_shared<FuzzyAssignment>(varEngine->getTable(), nameSpace,
				output, label);
}

//fuzzy predicates
NodePtr FuzzyBuilder::getPredicateInstance(string nameSpace,
			string predicateName, vector<Variable>& variables)
{
	PredicateInstance instance = predicateEngine->getPredicateInstance(
				nameSpace, predicateName, variables);

	for (size_t i = 0; i < variables.size(); i++)
	{
		varEngine->addDomains(variables[i].nameSpace, *instance.second[i]);
		varEngine->updateVariableMask(variables[i], ruleList->size());
	}

	return instance.first;
}

NodePtr FuzzyBuilder::getPredicateInstance(string predicateName,
			vector<Variable>& variables)
{
	return getPredicateInstance("", predicateName, variables);
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

