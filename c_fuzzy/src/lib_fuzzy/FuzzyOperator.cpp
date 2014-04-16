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

double FuzzyOr::evaluate(ReasoningData reasoningData)
{
	double a = leftOperand->evaluate(reasoningData);
	double b = rightOperand->evaluate(reasoningData);
	return (a > b) ? a : b;
}

double FuzzyNot::evaluate(ReasoningData reasoningData)
{
	return (1 - operand->evaluate(reasoningData));
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

BinaryFuzzyOperator::~BinaryFuzzyOperator()
{
	delete leftOperand;
	delete rightOperand;
}

