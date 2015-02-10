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

#ifndef TREECLASSIFIERBUILDER_H_
#define TREECLASSIFIERBUILDER_H_

#include <string>

#include "FuzzyClass.h"
#include "FuzzyClassifier.h"

class TreeClassifierBuilder
{

public:
	TreeClassifierBuilder();
	void parse(const char *filename);
	FuzzyClassifier* buildFuzzyClassifier();

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
	ConstantList* buildCostantList(ConstantList* list, std::string constant,
			std::string value);
	FuzzyConstraintsList* buildFeaturesList(FuzzyConstraintsList* list,
			std::vector<std::string>& labelList, FeatureType type);
	FuzzyConstraint* buildSimpleFeature(std::string variable,
			std::string fuzzyLabel);
	FuzzyConstraint* buildSimpleRelation(std::vector<std::string>& labelList);
	FuzzyConstraint* buildComplexRelation(std::vector<std::string>& labelList);
	FuzzyConstraint* buildInverseRelation(std::vector<std::string>& labelList);
	void buildClass(std::string name, std::string superClassName,
			VariableList* variables, ConstantList* constants,
			FuzzyConstraintsList* featureList, bool hidden);

private:
	void checkConsistency();
	void checkSuperClass(const std::string& name,
			const std::string& superClassName, FuzzyClass* superClass);
	void checkFeatureList(FuzzyClass& fuzzyClass);
	void checkVariable(FuzzyClass& fuzzyClass, std::string variable);
	void checkRelation(FuzzyClass& fuzzyClass, FuzzyConstraint& relation);
	void checkRelationVar(std::string relatedVariable, FuzzyClass& relatedClass,
			FuzzyClass& fuzzyClass);

private:
	//Data needed to track created classes
	FuzzyClassifier* classifier;

};

#endif /* TREECLASSIFIERBUILDER_H_ */
