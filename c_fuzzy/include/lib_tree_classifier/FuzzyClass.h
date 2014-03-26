/*
 * c_fuzzy,
 *
 *
 * Copyright (C) 2014 Davide Tateo
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

#ifndef FUZZYCLASS_H_
#define FUZZYCLASS_H_

#include <map>
#include <set>
#include <vector>

#include "FuzzyFeatures.h"

typedef std::set<std::string> VariableList;
typedef std::map<std::string, int> ConstantList;

class FuzzyClass
{
public:
	FuzzyClass(std::string name, FuzzyClass superClass, VariableList* variables,
				ConstantList* constants, FuzzyFeatureList* features,
				bool important) :
				name(name), superClass(superClass), variables(variables),
				constants(constants), features(features), important(important)
	{
	}

	~FuzzyClass()
	{
		delete variables;
		delete constants;
		delete features;
	}

private:
	std::string name;
	FuzzyClass& superClass;
	VariableList* variables;
	ConstantList* constants;
	FuzzyFeatureList* features;
	bool important;
};

#endif /* FUZZYCLASS_H_ */
