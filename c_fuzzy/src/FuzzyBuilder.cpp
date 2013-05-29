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

#include "FuzzyOperator.h"
#include "FuzzyBuilder.h"

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
	std::ifstream in_file(filename);
	if (!in_file.good())
		exit(-1);
	scanner = new FuzzyScanner(&in_file);
	/* check to see if its initialized */
	assert(scanner != NULL);
	parser = new yy::FuzzyParser((*scanner), (*this));
	assert(parser != NULL);
	if (parser->parse() == -1)
	{
		std::cerr << "Parse failed!!" << std::endl;
	}
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

FuzzyReasoner* FuzzyBuilder::createReasoner()
{
	return new FuzzyReasoner(inputTable, mfTable, aggregator, ruleList);
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

Node* FuzzyBuilder::buildIs(Node* left, Node* right)
{
	return new FuzzyIs(static_cast<CrispData*>(left),
			static_cast<MFLabel*>(right));
}

Node* FuzzyBuilder::buildNot(Node* operand)
{
	return new FuzzyNot(static_cast<FuzzyOperator*>(operand));
}

Node* FuzzyBuilder::buildAssignment(std::string* output, std::string* label)
{
	return new FuzzyAssignment(mfTable, aggregator, *output, *label);
}

//Fuzzy MF
void FuzzyBuilder::buildMF(std::string* name, std::string* shape, std::vector<int>* parameters)
{
	std::map<std::string, FuzzyMF*>& map = *mfTable;
	std::vector<int>& p = *parameters;

	//assert that the number of parameters matches the MF shape
	//then create the corrispondent MF labelled "name"
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
			map[*name] = buildTra(p[3],p[2], p[1], p[0]);
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

Node* FuzzyBuilder::buildMFLabel(std::string* label)
{
	//Makes sure that the Label has been defined
	assert(mfTable->count(*label) == 1);
	return new MFLabel(mfTable, *label);
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

//Fuzzy crisp Data
Node* FuzzyBuilder::buildCrispData(std::string* label)
{
	return new CrispData(inputTable, *label);
}

