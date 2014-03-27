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

#include "FuzzyClass.h"
#include "TreeClassifierScanner.h"
#include "TreeClassifierParser.tab.h"

class TreeClassifierBuilder
{

public:
	TreeClassifierBuilder() :
			parser(NULL), scanner(NULL)
	{
	}

	void parse(const char *filename);

	std::ostream& print(std::ostream &stream);

	virtual ~TreeClassifierBuilder();

private:
	template<class ListType>
	inline ListType* eventuallyInitialize(ListType* list)
	{
		if (list == NULL)
			return new ListType();
		else
			return list;
	}

public:
	VariableList* buildVariableList(VariableList* list, std::string variable);
	ConstantList* buildCostantList(ConstantList* list, std::string variable,
			std::string value);
	FuzzyFeatureList* buildFeaturesList(FuzzyFeatureList* list,
			std::vector<std::string>& labelList, FeatureType type);
	FuzzyFeature* buildSimpleFeature(std::string variable,
			std::string fuzzyLabel);
	FuzzyFeature* buildSimpleRelation(std::vector<std::string>& labelList);
	FuzzyFeature* buildComplexRelation(std::vector<std::string>& labelList);
	void buildClass(std::string name, std::string superClassName,
			VariableList* variables, ConstantList* constants,
			FuzzyFeatureList* featureList, bool important);

private:
	//Data needed to get Builder working
	tc::TreeClassifierParser* parser;
	TreeClassifierScanner* scanner;

	//Data needed to track created classes
	std::map<std::string, FuzzyClass*> classList;

};

#endif /* FUZZYBUILDER_H_ */
