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

#include <sstream>
#include <cmath>
#include <stdexcept>

using namespace std;

VariableGenerator::VariableGenerator()
{
	varCounter = 0;

	currentCandidates = NULL;
	currentDependecies = NULL;
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
	onVars[newVar] = OnVar(var, min, max);
	return newVar;
}

string VariableGenerator::addInverseOnVariable(Variable min, Variable max,
			Variable target)
{
	string newVar = getNewVar();
	inverseVars[newVar] = InverseVar(min, max, target);
	return newVar;
}

ObjectProperties VariableGenerator::getGeneratedProperties(
			ObjectMap& candidates, ObjectMap& dependencies)
{
	ObjectProperties generated;
	currentCandidates = &candidates;
	currentDependecies = &dependencies;
	generateMatches(generated);
	generateOns(generated);
	generateInverses(generated);

	currentCandidates = NULL;
	currentDependecies = NULL;

	return generated;

}

void VariableGenerator::generateMatches(ObjectProperties& generated)
{
	for (auto& it : matchVars)
	{

		string varName = it.first;
		MatchVar& match = it.second;
		Variable& var = match.var;
		Variable& target = match.target;

		int value = abs(getValue(var) - getDepValue(target));

		generated[varName] = value;
	}
}

void VariableGenerator::generateOns(ObjectProperties& generated)
{
	for (auto& it : onVars)
	{
		string varName = it.first;
		OnVar& on = it.second;
		Variable& var = on.var;
		Variable& min = on.min;
		Variable& max = on.max;

		int value = 100 * (getDepValue(var) - getValue(min))
					/ (getValue(max) - getValue(min));

		generated[varName] = value;
	}
}

void VariableGenerator::generateInverses(ObjectProperties& generated)
{
	for (auto& it : inverseVars)
	{
		string varName = it.first;
		InverseVar& inverse = it.second;
		Variable& target = inverse.target;
		Variable& min = inverse.min;
		Variable& max = inverse.max;

		int value = 100 * (getValue(target) - getDepValue(min))
					/ (getDepValue(max) - getDepValue(min));

		generated[varName] = value;
	}
}

int VariableGenerator::getValue(ObjectMap& inputs, Variable var)
{
	string& nameSpace = var.nameSpace;
	string& domain = var.domain;

	ObjectProperties& properties = inputs[nameSpace]->properties;

	return properties[domain];
}

int VariableGenerator::getValue(Variable var)
{
	return getValue(*currentCandidates, var);
}

int VariableGenerator::getDepValue(Variable var)
{
	string& className = var.nameSpace;

	if (currentDependecies->count(className) == 1)
		return getValue(*currentDependecies, var);
	else if (currentCandidates->count(className) == 1)
		return getValue(*currentCandidates, var);
	else
		throw runtime_error(
					"Error, no class object " + className + " found in inputs");

}

string VariableGenerator::getNewVar()
{
	stringstream ss;
	ss << "$" << ++varCounter;
	string newVar = ss.str();
	return newVar;
}

