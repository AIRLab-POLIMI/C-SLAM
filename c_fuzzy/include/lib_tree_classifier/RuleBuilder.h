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

#ifndef RULEBUILDER_H_
#define RULEBUILDER_H_

#include <string>
#include <utility>

#include "Node.h"
#include "FuzzyKnowledgeBase.h"
#include "FuzzyMFEngine.h"
#include "FuzzyClass.h"

#include "VariableGenerator.h"

class FuzzyClass;
class FuzzySimpleFeature;
class FuzzySimpleRelation;
class FuzzyComplexRelation;
class FuzzyInverseRelation;

class RuleBuilder
{
private:
	typedef std::pair<Node*, Variable> FeatureBuilt;
public:
	RuleBuilder(FuzzyKnowledgeBase& knowledgeBase);
	VariableGenerator* buildClassRule(FuzzyClass& fuzzyClass);

private:
	FeatureBuilt buildFeatureRule(FuzzyFeature& feature);
	FeatureBuilt buildSimpleFeatureRule(FuzzySimpleFeature& feature);
	FeatureBuilt buildSimpleRelationRule(FuzzySimpleRelation& relation);
	FeatureBuilt buildComplexRelationRule(FuzzyComplexRelation& relation);
	FeatureBuilt buildInverseRelationRule(FuzzyInverseRelation& relation);
	FeatureBuilt buildFeature(std::string generatedVar, std::string label,
				bool simple);
	Node* buildRHS();
	Node* buildCrispMatch(Variable var);
	Node* buildCrispOn(Variable var);
	void addDomain(std::string domain, std::string label, FuzzyMF* table);

private:
	FuzzyKnowledgeBase& knowledgeBase;

	//data needed to build rules
	std::string currentClass;

	VariableGenerator* generator;

};

#endif /* RULEBUILDER_H_ */
