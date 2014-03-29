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

#include <fstream>
#include <stdexcept>
#include <sstream>

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

	checkConsistency();
}

VariableList* TreeClassifierBuilder::buildVariableList(VariableList* list,
			string variable)
{

	list = eventuallyInitialize(list);

	pair<VariableList::iterator, bool> ret = list->insert(variable);

	if (!ret.second)
	{
		stringstream ss;
		ss << "Error: redeclaration of variable " << variable;
		throw runtime_error(ss.str());
	}

	return list;
}

ConstantList* TreeClassifierBuilder::buildCostantList(ConstantList* list,
			string constant, std::string value)
{
	list = eventuallyInitialize(list);

	if (list->count(constant) != 0)
	{
		stringstream ss;
		ss << "Error: redeclaration of constant " << constant;
		throw runtime_error(ss.str());
	}

	ConstantList& listRef = *list;

	listRef[constant] = value;

	return list;
}

FuzzyFeature* TreeClassifierBuilder::buildSimpleFeature(string variable,
			string fuzzyLabel)
{
	return new FuzzySimpleFeature(variable, fuzzyLabel);
}

FuzzyFeature* TreeClassifierBuilder::buildSimpleRelation(
			vector<string>& labelList)
{
	string className = labelList[0];
	string member = labelList[1];
	string matchingVar = labelList[2];

	if (labelList.size() == 3)
	{
		return new FuzzySimpleRelation(className, member, matchingVar);
	}
	else
	{
		string fuzzyLabel = labelList[3];
		return new FuzzySimpleRelation(className, member, matchingVar,
					fuzzyLabel);
	}
}

FuzzyFeature* TreeClassifierBuilder::buildComplexRelation(
			vector<string>& labelList)
{
	string className = labelList[0];
	string member = labelList[1];
	string variable1 = labelList[2];
	string variable2 = labelList[3];

	if (labelList.size() == 4)
	{
		return new FuzzyComplexRelation(className, member, variable1, variable2);
	}
	else
	{
		string fuzzyLabel = labelList[4];
		return new FuzzyComplexRelation(className, member, variable1, variable2,
					fuzzyLabel);
	}
}

FuzzyFeatureList* TreeClassifierBuilder::buildFeaturesList(
			FuzzyFeatureList* list, vector<string>& labelList, FeatureType type)
{
	FuzzyFeature* feature;

	list = eventuallyInitialize(list);

	switch (type)
	{
		case SIM_F:
			feature = buildSimpleFeature(labelList[0], labelList[1]);
			break;

		case SIM_R:
			feature = buildSimpleRelation(labelList);
			break;

		case COM_R:
			feature = buildComplexRelation(labelList);
			break;

		default:
			return list;
	}

	list->push_back(feature);

	return list;
}

void TreeClassifierBuilder::buildClass(string name, string superClassName,
			VariableList* variables, ConstantList* constants,
			FuzzyFeatureList* featureList, bool important)
{
	FuzzyClass* superClass = classList.find(superClassName)->second;

	checkSuperClass(name, superClassName, superClass);
	FuzzyClass* fuzzyClass = new FuzzyClass(name, superClass, variables,
				constants, featureList, important);
	classList[name] = fuzzyClass;
}

/* Consistency checks */
void TreeClassifierBuilder::checkSuperClass(const string& name,
			const string& superClassName, FuzzyClass* superClass)
{
	if (superClass == NULL && superClassName.compare("") != 0)
	{
		stringstream ss;
		ss << "Error: class " << name << " extends a non declared class "
					<< superClassName;
		throw runtime_error(ss.str());

	}
}

void TreeClassifierBuilder::checkConsistency()
{
	ClassList::iterator it;

	for (it = classList.begin(); it != classList.end(); ++it)
	{
		string name = it->first;
		FuzzyClass* fuzzyClass = it->second;
		checkFeatureList(*fuzzyClass);
	}
}

void TreeClassifierBuilder::checkFeatureList(FuzzyClass& fuzzyClass)
{
	FuzzyFeatureList* featuresPointer = fuzzyClass.getfeatureList();

	if (featuresPointer != NULL)
	{
		FuzzyFeatureList::iterator it;
		FuzzyFeatureList& features = *featuresPointer;
		for (it = features.begin(); it != features.end(); ++it)
		{
			FuzzyFeature& feature = **it;
			switch (feature.getFeatureType())
			{
				case SIM_F:
					checkVariable(fuzzyClass, feature.getVariables()[0]);
					break;

				case SIM_R:
					checkVariable(fuzzyClass, feature.getVariables()[0]);
					break;

				case COM_R:
					checkVariable(fuzzyClass, feature.getVariables()[0]);
					checkVariable(fuzzyClass, feature.getVariables()[1]);
					break;
			}
		}
	}
}

void TreeClassifierBuilder::checkVariable(FuzzyClass& fuzzyClass,
			string variable)
{
	if (!fuzzyClass.containsVar(variable))
	{
		stringstream ss;
		ss << "Error: rule in class " << fuzzyClass.getName()
					<< " references non existing variable " << variable;
		throw runtime_error(ss.str());
	}
}

TreeClassifierBuilder::~TreeClassifierBuilder()
{
	if (scanner != NULL)
		delete (scanner);
	if (parser != NULL)
		delete (parser);
}
