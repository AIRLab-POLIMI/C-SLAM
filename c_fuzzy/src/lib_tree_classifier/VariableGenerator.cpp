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

#include "VariableGenerator.h"

#include <strstream>

using namespace std;

VariableGenerator::VariableGenerator()
{
	varCounter = 0;
}

string VariableGenerator::addMatchVariable(Variable var, Variable target)
{
	string newVar = getNewVar();
	matchVars[newVar] = MatchVar(var, target);
	return newVar;
}

string VariableGenerator::addOnVariable(Variable var, Variable min,
			Variable max)
{
	string newVar = getNewVar();
	matchVars[newVar] = OnVar(var, min, max);
	return newVar;
}

string VariableGenerator::addInverseOnVariable(Variable min, Variable max,
			Variable target)
{
	string newVar = getNewVar();
	matchVars[newVar] = InverseVar(min, max, target);
	return newVar;
}

ObjectProperties VariableGenerator::getGeneratedProperties(InputClasses& inputs)
{
	ObjectProperties generated;
	generateMatches(inputs, generated);
	generateOns(inputs, generated);
	generateInverses(inputs, generated);

	return generated;

}

void VariableGenerator::generateMatches(InputClasses& inputs,
			ObjectProperties& generated)
{
	for (MatchVarMap::iterator it = matchVars.begin(); it != matchVars.end();
				++it)
	{
		string varName = it->first;
		MatchVar& match = it->second;
		Variable& var = match.var;
		Variable& target = match.target;

		int value = abs(getValue(inputs, var) - getValue(inputs, target));

		generated[varName] = value;
	}
}

void VariableGenerator::generateOns(InputClasses& inputs,
			ObjectProperties& generated)
{
	for (OnVarMap::iterator it = onVars.begin(); it != onVars.end(); ++it)
	{
		string varName = it->first;
		OnVar& on = it->second;
		Variable& var = on.var;
		Variable& min = on.min;
		Variable& max = on.max;

		int value = 100 * getValue(inputs, var)
					/ abs(getValue(inputs, max) - getValue(inputs, min));

		generated[varName] = value;
	}
}

void VariableGenerator::generateInverses(InputClasses& inputs,
			ObjectProperties& generated)
{
	for (InverseVarMap::iterator it = inverseVars.begin();
				it != inverseVars.end(); ++it)
	{
		string varName = it->first;
		InverseVar& inverse = it->second;
		Variable& target = inverse.target;
		Variable& min = inverse.min;
		Variable& max = inverse.max;

		int value = 100 * getValue(inputs, target)
					/ abs(getValue(inputs, max) - getValue(inputs, min));

		generated[varName] = value;
	}
}

int VariableGenerator::getValue(InputClasses& inputs, Variable var)
{
	string nameSpace = var.nameSpace;
	string domain = var.domain;

	ObjectProperties& properties = *inputs[nameSpace];

	return properties[domain];
}

string VariableGenerator::getNewVar()
{
	stringstream ss;
	ss << "$" << ++varCounter;
	string newVar = ss.str();
	return newVar;
}

