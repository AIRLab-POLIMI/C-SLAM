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

using namespace std;

FuzzyOperator::~FuzzyOperator()
{
}

BinaryFuzzyOperator::BinaryFuzzyOperator(NodePtr leftOperand,
			NodePtr rightOperand) :
			leftOperand(leftOperand), rightOperand(rightOperand)
{
}

void BinaryFuzzyOperator::findVariables(std::vector<Variable>& variables)
{
	leftOperand->findVariables(variables);
	rightOperand->findVariables(variables);
}

FuzzyAnd::FuzzyAnd(NodePtr left, NodePtr right) :
			BinaryFuzzyOperator(left, right)
{
}

double FuzzyAnd::evaluate(ReasoningData reasoningData)
{
	double a = leftOperand->evaluate(reasoningData);
	double b = rightOperand->evaluate(reasoningData);
	return (a < b) ? a : b;
}

NodePtr FuzzyAnd::instantiate(vector<Variable>& variables)
{
	NodePtr left = leftOperand->instantiate(variables);
	NodePtr right = rightOperand->instantiate(variables);
	return make_shared<FuzzyAnd>(left, right);
}

FuzzyOr::FuzzyOr(NodePtr left, NodePtr right) :
			BinaryFuzzyOperator(left, right)
{
}

double FuzzyOr::evaluate(ReasoningData reasoningData)
{
	double a = leftOperand->evaluate(reasoningData);
	double b = rightOperand->evaluate(reasoningData);
	return (a > b) ? a : b;
}

NodePtr FuzzyOr::instantiate(vector<Variable>& variables)
{
	NodePtr left = leftOperand->instantiate(variables);
	NodePtr right = rightOperand->instantiate(variables);
	return make_shared<FuzzyOr>(left, right);
}

FuzzyNot::FuzzyNot(NodePtr operand) :
			operand(operand)
{
}

double FuzzyNot::evaluate(ReasoningData reasoningData)
{
	return (1 - operand->evaluate(reasoningData));
}

NodePtr FuzzyNot::instantiate(vector<Variable>& variables)
{
	NodePtr op = operand->instantiate(variables);
	return make_shared<FuzzyNot>(op);
}

void FuzzyNot::findVariables(std::vector<Variable>& variables)
{
	operand->findVariables(variables);
}


FuzzyIs::FuzzyIs(NamespaceTable& lookUpTable, string nameSpace, string label,
			string mfLabel) :
			lookUpTable(lookUpTable), nameSpace(nameSpace), label(label),
			mfLabel(mfLabel)
{
}

double FuzzyIs::evaluate(ReasoningData reasoningData)
{
	DomainTable& map = *lookUpTable[nameSpace];
	MFTable& mfTable = *map[label];
	FuzzyMFPtr mFunction = mfTable[mfLabel];
	int crispValue = reasoningData.inputs[nameSpace][label];
	reasoningData.inputValue = crispValue;
	return mFunction->evaluate(reasoningData);
}

NodePtr FuzzyIs::instantiate(vector<Variable>& variables)
{
	return make_shared<FuzzyIs>(*this);
}

void FuzzyIs::findVariables(std::vector<Variable>& variables)
{
	variables.push_back(Variable(nameSpace, label));
}

FuzzyTemplateIs::FuzzyTemplateIs(NamespaceTable& lookUpTable,
			size_t templateVarIndex, string mfLabel) :
			lookUpTable(lookUpTable),
			templateVarIndex(templateVarIndex), mfLabel(mfLabel)
{
}

double FuzzyTemplateIs::evaluate(ReasoningData reasoningData)
{
	throw runtime_error("Evaluation of a non-instantiated template");
}

NodePtr FuzzyTemplateIs::instantiate(vector<Variable>& variables)
{
	return make_shared<FuzzyIs>(lookUpTable,
				variables[templateVarIndex].nameSpace,
				variables[templateVarIndex].domain, mfLabel);
}

void FuzzyTemplateIs::findVariables(std::vector<Variable>& variables)
{
	//NO operation to be done
}

FuzzyAssignment::FuzzyAssignment(NamespaceTable& lookUpTable, string nameSpace,
			string name, string mfLabel) :
			lookUpTable(lookUpTable), nameSpace(nameSpace), output(name),
			mfLabel(mfLabel)
{
}

double FuzzyAssignment::evaluate(ReasoningData reasoningData)
{
	double truthValue = reasoningData.truthValue;
	DomainTable& map = *lookUpTable[nameSpace];
	MFTable& mfTable = *map[output];
	FuzzyMFPtr mf = mfTable[mfLabel];
	double result = mf->defuzzify(truthValue);
	reasoningData.aggregator.addValue(nameSpace, output, mfLabel, truthValue,
				result);
	return result;
}

void FuzzyAssignment::findVariables(std::vector<Variable>& variables)
{
	throwUnimplementedException();
}

