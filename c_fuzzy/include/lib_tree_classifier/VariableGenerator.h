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

#ifndef VARIABLEGENERATOR_H_
#define VARIABLEGENERATOR_H_

#include <map>
#include <string>
#include "ClassificationData.h"

class VariableGenerator
{
private:
	struct MatchVar
	{
		MatchVar(Variable var, Variable target) :
					var(var), target(target)
		{
		}
		Variable var;
		Variable target;
	};

	struct OnVar
	{
		OnVar(Variable var, Variable min, Variable max) :
					var(var), min(min), max(max)
		{
		}
		Variable min;
		Variable max;
		Variable var;
	};

	struct InverseVar
	{
		InverseVar(Variable min, Variable max, Variable target) :
					min(min), max(max), target(target)
		{
		}
		Variable min;
		Variable max;
		Variable target;
	};

	typedef std::map<std::string, MatchVar> MatchVarMap;
	typedef std::map<std::string, OnVar> OnVarMap;
	typedef std::map<std::string, InverseVar> InverseVarMap;

public:
	VariableGenerator();
	std::string addMatchVariable(Variable var, Variable target);
	std::string addOnVariable(Variable var, Variable min, Variable max);
	std::string addInverseOnVariable(Variable min, Variable max,
				Variable target);

	ObjectProperties getGeneratedProperties(InputClasses& inputs);

private:
	void generateMatches(InputClasses& inputs, ObjectProperties& generated);
	void generateOns(InputClasses& inputs, ObjectProperties& generated);
	void generateInverses(InputClasses& inputs, ObjectProperties& generated);
	int getValue(InputClasses& inputs, Variable var);
	std::string getNewVar();
private:
	MatchVarMap matchVars;
	OnVarMap onVars;
	InverseVarMap inverseVars;
	size_t varCounter;

};

typedef std::map<std::string, VariableGenerator*> GeneratedVarTable;

#endif /* VARIABLEGENERATOR_H_ */
