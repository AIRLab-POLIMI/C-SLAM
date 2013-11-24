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

#ifndef FUZZYOPERATOR_H_
#define FUZZYOPERATOR_H_

#include "Node.h"
#include "FuzzyMF.h"
#include "CrispData.h"
#include "FuzzyAggregator.h"


/**
 * Abstract class for a generic fuzzy operator
 *
 *
 */
class FuzzyOperator: public Node
{
public:
	virtual ~FuzzyOperator()
	{
	}
};

/**
 * Abstract class for a generic Binary fuzzy operator
 *
 *
 */
class BinaryFuzzyOperator: public FuzzyOperator
{
public:
	BinaryFuzzyOperator(Node* leftOperand, Node* rightOperand) :
			leftOperand(leftOperand), rightOperand(rightOperand)
	{
	}
	virtual ~BinaryFuzzyOperator();

protected:
	Node* leftOperand;
	Node* rightOperand;
};

/**
 * The Fuzzy And operator
 * Implements a T-Norm operator
 *
 */
class FuzzyAnd: public BinaryFuzzyOperator
{
public:
	FuzzyAnd(FuzzyOperator* left, FuzzyOperator* right) :
			BinaryFuzzyOperator(left, right)
	{
	}
	double evaluate();
};

/**
 * The Fuzzy Or Operator
 * Implements a S-Norm operator
 *
 */
class FuzzyOr: public BinaryFuzzyOperator
{
public:
	FuzzyOr(FuzzyOperator* left, FuzzyOperator* right) :
			BinaryFuzzyOperator(left, right)
	{
	}
	double evaluate();
};

/**
 * The Fuzzy Not operations
 * Implements the fuzzy set negation
 *
 */
class FuzzyNot: public FuzzyOperator
{
public:
	FuzzyNot(FuzzyOperator* operand) :
			operand(operand)
	{
	}
	double evaluate();
	~FuzzyNot();

private:
	FuzzyOperator* operand;
};

/**
 * The IS operator
 * returns the membership value of a crisp data to a given fuzzy set
 *
 */
class FuzzyIs: public FuzzyOperator
{
public:
	FuzzyIs(DomainTable* lookUpTable, CrispData* crispData, std::string mfLabel) :
			lookUpTable(*lookUpTable), crispData(crispData), mfLabel(mfLabel)
	{
	}
	double evaluate();
	~FuzzyIs();

private:
	CrispData* crispData;
	DomainTable& lookUpTable;
	std::string mfLabel;
};

/**
 * Fuzzy assignment operator.
 * Assign a semantic label to an output
 *
 */
class FuzzyAssignment: public FuzzyOperator
{
public:
	FuzzyAssignment(DomainTable* lookUpTable, FuzzyAggregator* aggregator,
			std::string name, std::string mfLabel) :
			lookUpTable(*lookUpTable), aggregator(*aggregator), mfLabel(
					mfLabel), output(name)
	{
	}
	double evaluate(double value);

private:
	DomainTable& lookUpTable;
	FuzzyAggregator& aggregator;
	std::string mfLabel;
	std::string output;
};

#endif /* FUZZYOPERATOR_H_ */