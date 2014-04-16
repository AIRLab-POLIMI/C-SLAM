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
#include <sstream>
#include <stdexcept>

#include "FuzzyOperator.h"
#include "FuzzyBuilder.h"

using namespace std;

FuzzyBuilder::~FuzzyBuilder()
{
	if (scanner != NULL)
		delete (scanner);
	if (parser != NULL)
		delete (parser);
}

void FuzzyBuilder::parse(const char *filename)
{
	ifstream inputFile(filename);
	if (!inputFile.good())
	{
		throw runtime_error("Bad file to parse");
	}
	scanner = new fz::FuzzyScanner(&inputFile);
	parser = new fz::FuzzyParser(*this, *scanner);

	if (parser->parse() == -1)
	{
		throw runtime_error("Parse Failed");
	}
}

FuzzyKnowledgeBase* FuzzyBuilder::createKnowledgeBase()
{
	namespaceMasks->normalizeVariableMasks(ruleList->size());
	return new FuzzyKnowledgeBase(namespaceTable, namespaceMasks, ruleList);
}

void FuzzyBuilder::buildRule(Node* antecedent, Node* conseguent)
{
	Node* rule = new FuzzyRule(antecedent, conseguent);
	ruleList->push_back(rule);
}

//Predicates
void FuzzyBuilder::buildPredicate(string predicateName, Node* definition)
{
	//FIXME implementami
}

//Fuzzy operators
Node* FuzzyBuilder::buildAnd(Node* left, Node* right)
{
	return new FuzzyAnd(static_cast<FuzzyOperator*>(left),
				static_cast<FuzzyOperator*>(right));
}

Node* FuzzyBuilder::buildOr(Node* left, Node* right)
{
	return new FuzzyOr(static_cast<FuzzyOperator*>(left),
				static_cast<FuzzyOperator*>(right));
}

Node* FuzzyBuilder::buildNot(Node* operand)
{
	return new FuzzyNot(static_cast<FuzzyOperator*>(operand));
}

Node* FuzzyBuilder::buildIs(string domain, string mfLabel)
{
	VariableMasks& mask = getVariableMasks("");
	mask.updateVariableMask(domain, ruleList->size());
	return new FuzzyIs(namespaceTable, "", domain, mfLabel);
}

Node* FuzzyBuilder::buildIs(pair<string, string> classMember, string mfLabel)
{
	string nameSpace = classMember.first;
	string domain = classMember.second;
	VariableMasks& mask = getVariableMasks(nameSpace);
	mask.updateVariableMask(domain, ruleList->size());
	return new FuzzyIs(namespaceTable, nameSpace, domain, mfLabel);
}

Node* FuzzyBuilder::buildTemplateIs(string domain, string mfLabel)
{
	//FIXME errore! implementare
	//variableMasks->updateVariableMask(domain, ruleList->size());
	//return new FuzzyIs(namespaceTable, "", domain, mfLabel);
	return NULL;
}

Node* FuzzyBuilder::buildAssignment(string output, string label)
{
	return new FuzzyAssignment(namespaceTable, "", output, label);
}

Node* FuzzyBuilder::buildAssignment(pair<string, string> classMember,
			string label)
{
	string nameSpace = classMember.first;
	string output = classMember.second;
	return new FuzzyAssignment(namespaceTable, nameSpace, output, label);
}

//Function to enter a namespace
void FuzzyBuilder::setNameSpace(string nameSpace)
{
	NamespaceTable& namespaceMap = *namespaceTable;
	NamespaceMasks& masksMap = *namespaceMasks;

	if (namespaceMap.count(nameSpace) == 0)
	{
		domainTable = new DomainTable();
		namespaceMap[nameSpace] = domainTable;
		variableMasks = new VariableMasks();
		namespaceMasks->addNameSpace(nameSpace, variableMasks);
	}
	else
	{
		domainTable = namespaceMap[nameSpace];
		variableMasks = masksMap[nameSpace];
	}
}

//Function to return to the default namespace
void FuzzyBuilder::setDefaultNameSpace()
{
	setNameSpace("");
}

//Fuzzy domain
void FuzzyBuilder::buildDomain(string variable)
{
	vector<string> variableVector;
	variableVector.push_back(variable);
	buildDomain(variableVector);
}

void FuzzyBuilder::buildDomain(vector<string> variables)
{
	DomainTable& domainMap = *domainTable;
	mfTable = new MFTable();
	vector<string>::iterator it;
	for (it = variables.begin(); it != variables.end(); it++)
	{
		domainMap[*it] = mfTable;
		variableMasks->newVariableMask(*it);
	}
}

//Fuzzy MF
void FuzzyBuilder::buildMF(string name, string shape, vector<int>& parameter)
{
	MFTable& map = *mfTable;
	FuzzySets fuzzySetType = fuzzyMap[shape];

	switch (fuzzySetType)
	{
		case TOL:
			checkParameters(name, parameter, fuzzySetType);
			map[name] = buildTol(parameter[1], parameter[0]);
			break;
		case TOR:
			checkParameters(name, parameter, fuzzySetType);
			map[name] = buildTor(parameter[1], parameter[0]);
			break;
		case TRA:
			checkParameters(name, parameter, fuzzySetType);
			map[name] = buildTra(parameter[3], parameter[2], parameter[1],
						parameter[0]);
			break;
		case TRI:
			checkParameters(name, parameter, fuzzySetType);
			map[name] = buildTri(parameter[2], parameter[1], parameter[0]);
			break;
		case INT:
			checkParameters(name, parameter, fuzzySetType);
			map[name] = buildInt(parameter[1], parameter[0]);
			break;
		case SGT:
			checkParameters(name, parameter, fuzzySetType);
			map[name] = buildSgt(parameter[0]);
			break;
		default:
			break;
	}
}

FuzzyMF* FuzzyBuilder::buildTor(int bottom, int top)
{
	return new TorMF(bottom, top);
}

FuzzyMF* FuzzyBuilder::buildTol(int top, int bottom)
{
	return new TolMF(top, bottom);
}

FuzzyMF* FuzzyBuilder::buildTra(int bottomLeft, int topLeft, int topRight,
			int bottomRight)
{
	return new TraMF(bottomLeft, topLeft, topRight, bottomRight);
}

FuzzyMF* FuzzyBuilder::buildTri(int left, int center, int right)
{
	return new TriMF(left, center, right);
}

FuzzyMF* FuzzyBuilder::buildInt(int left, int right)
{
	return new IntMF(left, right);
}

FuzzyMF* FuzzyBuilder::buildSgt(int value)
{
	return new SgtMF(value);
}

void FuzzyBuilder::createMap()
{
	fuzzyMap["tol"] = TOL;
	fuzzyMap["tor"] = TOR;
	fuzzyMap["tra"] = TRA;
	fuzzyMap["tri"] = TRI;
	fuzzyMap["int"] = INT;
	fuzzyMap["sgt"] = SGT;

}

void FuzzyBuilder::initializeNameSpaces()
{
	namespaceTable = new NamespaceTable();
	namespaceMasks = new NamespaceMasks();
	domainTable = new DomainTable();
	mfTable = NULL;

	NamespaceTable& namespaceMap = *namespaceTable;
	namespaceMap[""] = domainTable;

	namespaceMasks = new NamespaceMasks();
	variableMasks = new VariableMasks();

	namespaceMasks->addNameSpace("", variableMasks);

}

VariableMasks& FuzzyBuilder::getVariableMasks(std::string nameSpace)
{
	NamespaceMasks& nMask = *namespaceMasks;
	return *nMask[nameSpace];
}

void FuzzyBuilder::chekParametersNumber(string name, FuzzySets fuzzySetType,
			size_t parametersSize)
{
	bool error = false;

	switch (fuzzySetType)
	{
		case SGT:
			if (parametersSize != 1)
				error = true;
			break;
		case TOL:
		case TOR:
		case INT:
			if (parametersSize != 2)
				error = true;
			break;
		case TRI:
			if (parametersSize != 3)
				error = true;
			break;
		case TRA:
			if (parametersSize != 4)
				error = true;
			break;
	}

	if (error)
	{
		stringstream ss;
		ss << "The parameters number is wrong for label " << name;
		throw runtime_error(ss.str());
	}
}

void FuzzyBuilder::checkParameters(string name, vector<int>& parameters,
			FuzzySets fuzzySetType)
{
	size_t parametersSize = parameters.size();
	chekParametersNumber(name, fuzzySetType, parametersSize);

	for (size_t i = 1; i < parametersSize; i++)
	{
		if (parameters[i] > parameters[i - 1])
		{
			stringstream ss;
			ss << "The parameters ordering is wrong for label " << name;
			throw runtime_error(ss.str());
		}
	}
}

