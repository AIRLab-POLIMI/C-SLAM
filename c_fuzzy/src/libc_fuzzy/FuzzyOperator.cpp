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



double FuzzyAnd::evaluate(InputTable inputs)
{
	double a = leftOperand->evaluate(inputs);
	double b = rightOperand->evaluate(inputs);
	return (a < b) ? a : b;
}

double FuzzyOr::evaluate(InputTable inputs)
{
	double a = leftOperand->evaluate(inputs);
	double b = rightOperand->evaluate(inputs);
	return (a > b) ? a : b;
}

double FuzzyNot::evaluate(InputTable inputs)
{
	return (1 - operand->evaluate(inputs));
}

FuzzyNot::~FuzzyNot()
{
	delete operand;
}


double FuzzyAssignment::evaluate(double value)
{
	MFTable& mfTable = *lookUpTable[output];
	FuzzyMF* mf = mfTable[mfLabel];
	double result = mf->defuzzify(value);
	aggregator.addValue(output, mfLabel, value, result);
	return result;
}


double FuzzyIs::evaluate(InputTable inputs)
{
	MFTable& mfTable = *lookUpTable[label];
	Node* mFunction = mfTable[mfLabel];
	int crispValue = inputs[label];
	return mFunction->evaluate(crispValue);
}


BinaryFuzzyOperator::~BinaryFuzzyOperator()
{
	delete leftOperand;
	delete rightOperand;
}


