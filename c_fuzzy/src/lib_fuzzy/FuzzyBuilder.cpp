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
#include <cassert>
#include <stdexcept>

#include "FuzzyOperator.h"
#include "FuzzyBuilder.h"

using namespace std;

//TODO eliminare assertion e lanciare eccezioni...

FuzzyBuilder::~FuzzyBuilder()
{
	if (scanner != NULL)
		delete (scanner);
	if (parser != NULL)
		delete (parser);
}

void FuzzyBuilder::parse(const char *filename)
{
	assert(filename != NULL);
	ifstream inputFile(filename);
	if (!inputFile.good())
	{
		throw runtime_error("Bad file to parse");
	}
	scanner = new FuzzyScanner(&inputFile);
	/* check to see if its initialized */
	assert(scanner != NULL);
	parser = new yy::FuzzyParser((*scanner), (*this));
	assert(parser != NULL);
	if (parser->parse() == -1)
	{
		throw runtime_error("Parse Failed");
	}
}

FuzzyKnowledgeBase* FuzzyBuilder::createKnowledgeBase()
{
	normalizeVariableMasks();
	return new FuzzyKnowledgeBase(domainTable, variableMasks, ruleList);
}

void FuzzyBuilder::buildRule(Node* antecedent, Node* conseguent)
{
	Node* rule = new FuzzyRule(antecedent, conseguent);
	ruleList->push_back(rule);
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

Node* FuzzyBuilder::buildIs(string* domain, string* mfLabel)
{
	//TODO assertion to check existence of all the stuff...
	updateVariableMask(*domain);
	return new FuzzyIs(domainTable, *domain, *mfLabel);
}

Node* FuzzyBuilder::buildAssignment(string* output, string* label)
{
	return new FuzzyAssignment(domainTable, *output, *label);
}

//Fuzzy domain
void FuzzyBuilder::buildDomain(vector<string> variables)
{
	DomainTable& domainMap = *domainTable;
	map<string, BitData>& maskMap = *variableMasks;
	mfTable = new MFTable();
	vector<string>::iterator it;
	for (it = variables.begin(); it != variables.end(); it++)
	{
		domainMap[*it] = mfTable;
		maskMap[*it] = BitData(maskMap.size(), new boost::dynamic_bitset<>());
	}
}

//Fuzzy MF
void FuzzyBuilder::buildMF(string* name, string* shape, vector<int>* parameters)
{
	MFTable& map = *mfTable;
	vector<int>& p = *parameters;

	//assert that the number of parameters matches the MF shape
	//then create the correspondent MF labeled "name"
	switch (fuzzyMap[*shape])
	{
		case TOL:
			assert(p.size() == 2);
			assert(p[1] < p[0]);
			map[*name] = buildTol(p[1], p[0]);
			break;
		case TOR:
			assert(p.size() == 2);
			assert(p[1] < p[0]);
			map[*name] = buildTor(p[1], p[0]);
			break;
		case TRA:
			assert(p.size() == 4);
			assert(p[3] < p[2] && p[2] < p[1] && p[1] < p[0]);
			map[*name] = buildTra(p[3], p[2], p[1], p[0]);
			break;
		case TRI:
			assert(p.size() == 3);
			assert(p[2] < p[1] && p[1] < p[0]);
			map[*name] = buildTri(p[2], p[1], p[0]);
			break;
		case INT:
			assert(p.size() == 2);
			assert(p[1] < p[0]);
			map[*name] = buildInt(p[1], p[0]);
			break;
		case SGT:
			assert(p.size() == 1);
			map[*name] = buildSgt(p[0]);
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

void FuzzyBuilder::updateVariableMask(string& label)
{
	size_t currentRule = ruleList->size();
	map<string, BitData>& maskMap = *variableMasks;
	boost::dynamic_bitset<>& bitset = *maskMap[label].bits;
	if (currentRule >= bitset.size())
		bitset.resize(currentRule + 1, false);

	bitset[currentRule] = true;
}

void FuzzyBuilder::normalizeVariableMasks()
{
	map<string, BitData>::iterator it;
	for (it = variableMasks->begin(); it != variableMasks->end(); ++it)
	{
		it->second.bits->resize(ruleList->size(), false);
	}

}
