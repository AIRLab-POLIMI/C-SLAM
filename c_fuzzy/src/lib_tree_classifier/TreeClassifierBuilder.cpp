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

#include "TreeClassifierBuilder.h"

using namespace std;

VariableList* TreeClassifierBuilder::buildVariableList(VariableList* list,
			string variable)
{

	if (list == NULL)
	{
		list = new VariableList();
	}

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
	if (list == NULL)
	{
		list = new ConstantList();
	}

	if (list->count(variable) != 0)
	{
		//TODO error redeclaration of constant
	}

	list->at(variable) = value;

	return list;
}

FuzzyFeatureList* TreeClassifierBuilder::buildFeaturesList(
			FuzzyFeatureList* list, vector<string>* labelList)
{

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
