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

#include "RuleBuilder.h"

#include <stdexcept>

#include "FuzzyOperator.h"
#include "FuzzyRule.h"

using namespace std;

RuleBuilder::RuleBuilder(FuzzyKnowledgeBase& knowledgeBase) :
			knowledgeBase(knowledgeBase), generator(NULL)
{
}

VariableGenerator* RuleBuilder::buildClassRule(FuzzyClass& fuzzyClass)
{
	fixNameSpace(fuzzyClass);
	currentClass = fuzzyClass.getName();
	generator = new VariableGenerator();

	Node* lhs;
	Node* rhs;

	vector<Variable> variables;

	FuzzyFeatureList& features = *fuzzyClass.getfeatureList();

	FuzzyFeatureList::reverse_iterator start = features.rbegin();
	FuzzyFeatureList::reverse_iterator end = features.rend();

	for (FuzzyFeatureList::reverse_iterator i = start; i != end; ++i)
	{
		FuzzyFeature* feature = *i;
		FeatureBuilt featureBuilt = buildFeatureRule(*feature);
		Node* featureRule = featureBuilt.first;
		variables.push_back(featureBuilt.second);
		if (i == start)
		{
			lhs = featureRule;
		}
		else
		{
			lhs = new FuzzyAnd(featureRule, lhs);
		}

	}

	rhs = buildRHS();
	Node* rule = new FuzzyRule(lhs, rhs);

	knowledgeBase.addRule(rule, variables);

	return generator;
}

void RuleBuilder::fixNameSpace(FuzzyClass& fuzzyClass)
{
	FuzzyClass* superClass = fuzzyClass.getSuperClass();

	if (superClass)
	{
		string className = fuzzyClass.getName();
		string superClassName = superClass->getName();

		NamespaceTable& table = knowledgeBase.getNamespaceTable();

		knowledgeBase.addDomains(className, *table[superClassName]);
	}
}

RuleBuilder::FeatureBuilt RuleBuilder::buildFeatureRule(FuzzyFeature& feature)
{
	switch (feature.getFeatureType())
	{
		case SIM_F:
			return buildSimpleFeatureRule(
						static_cast<FuzzySimpleFeature&>(feature));
		case SIM_R:
			return buildSimpleRelationRule(
						static_cast<FuzzySimpleRelation&>(feature));
		case COM_R:
			return buildComplexRelationRule(
						static_cast<FuzzyComplexRelation&>(feature));
		case INV_R:
			return buildInverseRelationRule(
						static_cast<FuzzyInverseRelation&>(feature));
		default:
			return FeatureBuilt(NULL, Variable("", ""));
	}
}

RuleBuilder::FeatureBuilt RuleBuilder::buildSimpleFeatureRule(
			FuzzySimpleFeature& feature)
{
	string varName = feature.getVariables().back();
	string label = feature.getFuzzyLabel();

	Node* is = new FuzzyIs(knowledgeBase.getNamespaceTable(), currentClass,
				varName, label);
	Variable var(currentClass, varName);

	return FeatureBuilt(is, var);

}

RuleBuilder::FeatureBuilt RuleBuilder::buildSimpleRelationRule(
			FuzzySimpleRelation& relation)
{
	string varName = relation.getVariables().back();
	string label = relation.getFuzzyLabel();
	string relatedClass = relation.getRelationObject();
	string relatedVar = relation.getRelationVariable();

	Variable variable(currentClass, varName);
	Variable target(relatedClass, relatedVar);
	string generatedVar = generator->addMatchVariable(variable, target);

	Variable var(currentClass, generatedVar);
	Node* node;

	return buildFeature(generatedVar, label, true);
}

RuleBuilder::FeatureBuilt RuleBuilder::buildComplexRelationRule(
			FuzzyComplexRelation& relation)
{
	string varMin = relation.getVariables()[0];
	string varMax = relation.getVariables()[1];
	string label = relation.getFuzzyLabel();
	string relatedClass = relation.getRelationObject();
	string relatedVar = relation.getRelationVariable();

	Variable min(currentClass, varMin);
	Variable max(currentClass, varMax);
	Variable target(relatedClass, relatedVar);
	string generatedVar = generator->addOnVariable(target, min, max);

	return buildFeature(generatedVar, label, false);
}

RuleBuilder::FeatureBuilt RuleBuilder::buildInverseRelationRule(
			FuzzyInverseRelation& relation)
{
	string variable = relation.getRelationVariable();
	string label = relation.getFuzzyLabel();

	string relatedClass = relation.getRelationObject();
	string varMin = relation.getVariables()[0];
	string varMax = relation.getVariables()[1];

	Variable min(relatedClass, varMin);
	Variable max(relatedClass, varMax);
	Variable target(currentClass, variable);
	string generatedVar = generator->addInverseOnVariable(min, max, target);

	return buildFeature(generatedVar, label, false);
}

RuleBuilder::FeatureBuilt RuleBuilder::buildFeature(string& generatedVar,
			string& label, bool simple)
{
	Variable var(currentClass, generatedVar);
	Node* node;

	if (!label.empty())
	{
		node = knowledgeBase.getPredicateInstance(currentClass, label, var);
	}
	else if (simple)
	{
		node = buildCrispMatch(var);
	}
	else
	{
		node = buildCrispOn(var);
	}

	return FeatureBuilt(node, var);
}

Node* RuleBuilder::buildRHS()
{
	addDomain(currentClass, "True", FuzzyMFEngine::buildSgt(1));
	return new FuzzyAssignment(knowledgeBase.getNamespaceTable(), currentClass,
				currentClass, "True");
}

Node* RuleBuilder::buildCrispMatch(Variable var)
{
	addDomain(var.domain, "Perfect", FuzzyMFEngine::buildSgt(0));

	Node* is = new FuzzyIs(knowledgeBase.getNamespaceTable(), var.nameSpace,
				var.domain, "Perfect");

	return is;
}

Node* RuleBuilder::buildCrispOn(Variable var)
{
	addDomain(var.domain, "Into", FuzzyMFEngine::buildInt(0, 100));

	Node* is = new FuzzyIs(knowledgeBase.getNamespaceTable(), var.nameSpace,
				var.domain, "Into");

	return is;
}

void RuleBuilder::addDomain(const string& domain, const string& label,
			FuzzyMF* fuzzyMF)
{
	MFTable* mfTable = new MFTable();
	MFTable& table = *mfTable;
	table[label] = fuzzyMF;

	DomainTable domainTable;
	domainTable[currentClass] = mfTable;

	knowledgeBase.addDomains(currentClass, domainTable);

}

