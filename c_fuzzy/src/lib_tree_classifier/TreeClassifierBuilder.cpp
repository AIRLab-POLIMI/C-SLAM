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

#include "TreeClassifierBuilder.h"

using namespace std;

void TreeClassifierBuilder::parse(const char *filename)
{
	ifstream inputFile(filename);
	if (!inputFile.good())
	{
		throw runtime_error("Bad file to parse");
	}
	scanner = new TreeClassifierScanner(&inputFile);
	parser = new tc::TreeClassifierParser((*scanner), (*this));
	if (parser->parse() == -1)
	{
		throw runtime_error("Parse Failed");
	}
}

VariableList* TreeClassifierBuilder::buildVariableList(VariableList* list,
			string variable)
{

	list = eventuallyInitialize(list);

	pair<VariableList::iterator, bool> ret = list->insert(variable);

	if (!ret.second)
	{
		//TODO error redeclaration of variable
	}

	return list;
}

ConstantList* TreeClassifierBuilder::buildCostantList(ConstantList* list,
			string variable, int value)
{
	list = eventuallyInitialize(list);

	if (list->count(variable) != 0)
	{
		//TODO error redeclaration of constant
	}

	list->at(variable) = value;

	return list;
}

FuzzyFeature* TreeClassifierBuilder::buildFeature(string variable,
			string fuzzyLabel)
{
	return new FuzzySimpleFeature(variable, fuzzyLabel);
}

FuzzyFeature* TreeClassifierBuilder::buildFeature(vector<string>& labelList)
{
	string className = labelList[0];
	string member = labelList[1];
	string variable1 = labelList[2];
	string variable2 = labelList[3];

	if (labelList.size() == 4)
	{
		return new FuzzyRelation(className, member, variable1, variable2);
	}
	else
	{
		string fuzzyLabel = labelList[4];
		return new FuzzyRelation(className, member, variable1, variable2,
					fuzzyLabel);
	}
}

FuzzyFeatureList* TreeClassifierBuilder::buildFeaturesList(
			FuzzyFeatureList* list, vector<string>& labelList)
{
	FuzzyFeature* feature;

	list = eventuallyInitialize(list);

	if (list->size() == 2)
		feature = buildFeature(labelList[0], labelList[1]);
	else
		feature = buildFeature(labelList);

	list->push_back(feature);

	return list;
}

void TreeClassifierBuilder::buildClass(string name, string superClassName,
			VariableList* variables, ConstantList* constants,
			FuzzyFeatureList* featureList, bool important)
{
	FuzzyClass& superClass = *classList[superClassName];
	FuzzyClass* fuzzyClass = new FuzzyClass(name, superClass, variables,
				constants, featureList, important);
	classList[name] = fuzzyClass;
}

TreeClassifierBuilder::~TreeClassifierBuilder()
{
	if (scanner != NULL)
		delete (scanner);
	if (parser != NULL)
		delete (parser);
}
