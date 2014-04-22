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

#include "FuzzyOperator.h"

using namespace std;

RuleBuilder::RuleBuilder(FuzzyKnowledgeBase& knowledgeBase) :
		knowledgeBase(knowledgeBase), generator(NULL)
{
}

VariableGenerator* RuleBuilder::buildClassRule(FuzzyClass& fuzzyClass)
{
	currentClass = fuzzyClass.getName();
	generator = new VariableGenerator();

	Node* lhs;

	FuzzyFeatureList& features = *fuzzyClass.getfeatureList();

	FuzzyFeatureList::reverse_iterator start = features.rbegin();
	FuzzyFeatureList::reverse_iterator end = features.rend();

	for (FuzzyFeatureList::reverse_iterator i = start;
			i != end; ++i)
	{
		FuzzyFeature* feature = *i;
		Node* featureRule = buildFeatureRule(*feature);

		if (i == start)
		{
			lhs = featureRule;
		}
		else
		{
			lhs = new FuzzyAnd(featureRule, lhs);
		}

	}

	//FIXME add rule...

	return generator;
}

Node* RuleBuilder::buildFeatureRule(FuzzyFeature& feature)
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
			return NULL;
	}
}

Node* RuleBuilder::buildSimpleFeatureRule(FuzzySimpleFeature& feature)
{
	string varName = feature.getVariables().back();
	string label = feature.getFuzzyLabel();

	return new FuzzyIs(knowledgeBase.getNamespaceTable(), currentClass, varName,
			label);

}

Node* RuleBuilder::buildSimpleRelationRule(FuzzySimpleRelation& relation)
{
	string varName = relation.getVariables().back();
	string label = relation.getFuzzyLabel();
	string relatedClass = relation.getRelationObject();
	string relatedVar = relation.getRelationVariable();

	Variable variable(currentClass, varName);
	Variable target(relatedClass, relatedVar);
	string generatedVar = generator->addMatchVariable(variable, target);

	if (!label.empty())
	{
		return knowledgeBase.getPredicateInstance(currentClass, label,
				Variable(currentClass, generatedVar));
	}
	else
	{
		//TODO implement
		return NULL;
	}
}

Node* RuleBuilder::buildComplexRelationRule(FuzzyComplexRelation& relation)
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

	if (!label.empty())
	{

		return knowledgeBase.getPredicateInstance(currentClass, label,
				Variable(currentClass, generatedVar));
	}
	else
	{
		//TODO implement
		return NULL;
	}
}

Node* RuleBuilder::buildInverseRelationRule(FuzzyInverseRelation& relation)
{
	string var = relation.getRelationVariable();
	string label = relation.getFuzzyLabel();

	string relatedClass = relation.getRelationObject();
	string varMin = relation.getVariables()[0];
	string varMax = relation.getVariables()[1];

	Variable min(relatedClass, varMin);
	Variable max(relatedClass, varMax);
	Variable target(currentClass, var);
	string generatedVar = generator->addInverseOnVariable(min, max, target);

	if (!label.empty())
	{
		return knowledgeBase.getPredicateInstance(currentClass, label,
				Variable(currentClass, generatedVar));
	}
	else
	{
		//TODO implement
		return NULL;
	}
}
