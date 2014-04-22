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

#include "FuzzyOperator.h"


double FuzzyAnd::evaluate(ReasoningData reasoningData)
{
	double a = leftOperand->evaluate(reasoningData);
	double b = rightOperand->evaluate(reasoningData);
	return (a < b) ? a : b;
}

Node* FuzzyAnd::instantiate(Variable variable)
{
	Node* left = leftOperand->instantiate(variable);
	Node* right = rightOperand->instantiate(variable);
	return new FuzzyAnd(left, right);
}

double FuzzyOr::evaluate(ReasoningData reasoningData)
{
	double a = leftOperand->evaluate(reasoningData);
	double b = rightOperand->evaluate(reasoningData);
	return (a > b) ? a : b;
}

Node* FuzzyOr::instantiate(Variable variable)
{
	Node* left = leftOperand->instantiate(variable);
	Node* right = rightOperand->instantiate(variable);
	return new FuzzyOr(left, right);
}

double FuzzyNot::evaluate(ReasoningData reasoningData)
{
	return (1 - operand->evaluate(reasoningData));
}

Node* FuzzyNot::instantiate(Variable variable)
{
	Node* op = operand->instantiate(variable);
	return new FuzzyNot(op);
}

FuzzyNot::~FuzzyNot()
{
	delete operand;
}

double FuzzyAssignment::evaluate(ReasoningData reasoningData)
{
	double truthValue = reasoningData.truthValue;
	DomainTable& map = *lookUpTable[nameSpace];
	MFTable& mfTable = *map[output];
	FuzzyMF* mf = mfTable[mfLabel];
	double result = mf->defuzzify(truthValue);
	reasoningData.aggregator.addValue(nameSpace, output, mfLabel, truthValue,
				result);
	return result;
}

double FuzzyIs::evaluate(ReasoningData reasoningData)
{
	DomainTable& map = *lookUpTable[nameSpace];
	MFTable& mfTable = *map[label];
	Node* mFunction = mfTable[mfLabel];
	int crispValue = reasoningData.inputs[nameSpace][label];
	reasoningData.inputValue = crispValue;
	return mFunction->evaluate(reasoningData);
}

Node* FuzzyIs::instantiate(Variable variable)
{
	return new FuzzyIs(*this);
}

double FuzzyTemplateIs::evaluate(ReasoningData reasoningData)
{
	throw std::logic_error("Evaluation of a non-instantiated template");
}

Node* FuzzyTemplateIs::instantiate(Variable variable)
{
	return new FuzzyIs(lookUpTable, variable.nameSpace, variable.domain, mfLabel);
}

BinaryFuzzyOperator::~BinaryFuzzyOperator()
{
	delete leftOperand;
	delete rightOperand;
}

