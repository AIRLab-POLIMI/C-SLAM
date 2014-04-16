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

#ifndef FUZZYREASONER_H_
#define FUZZYREASONER_H_

#include <map>
#include <string>
#include <vector>

#include <boost/dynamic_bitset.hpp>

#include "FuzzyKnowledgeBase.h"
#include "ReasoningData.h"

/**
 * This struct is used to store the couple value/truth level
 *
 */
struct FuzzyOutput
{
	double value;
	double truth;
};

/*
 * Type to store the corresponding domain outputs
 */
typedef std::map<std::string, FuzzyOutput> DomainOutputTable;

/*
 * Type to store the corresponding namespace outputs
 */
typedef std::map<std::string, DomainOutputTable> OutputTable;

/**
 * The class implementing the defuzzyfier
 * this is intended to be an extensible defuzzifyer, but for now, implementing the centroid
 * only with singletons.
 */
class Defuzzyfier
{
public:
	OutputTable defuzzify(AggregationMap& aggregatedData);
};

/**
 * The class implementing the reasoner.
 *
 *
 */
class FuzzyReasoner
{
public:
	FuzzyReasoner(FuzzyKnowledgeBase& knowledgeBase) :
				knowledgeBase(knowledgeBase),
				namespaceMasks(knowledgeBase.getNamespaceMasks())
	{
		rulesMask.resize(knowledgeBase.size(), false);
		inputMask.resize(namespaceMasks.variablesNumber(), false);

		rulesMask.reset();
		inputMask.reset();
	}
	void addInput(std::string nameSpace, std::string name, int value);
	void addInput(std::string name, int value);
	OutputTable run();

private:
	void updateRulesMask();
	void cleanInputData();

private:
	FuzzyKnowledgeBase& knowledgeBase;
	FuzzyAggregator aggregator;
	Defuzzyfier defuzzyfier;
	InputTable inputs;
	NamespaceMasks& namespaceMasks;
	boost::dynamic_bitset<> rulesMask;
	boost::dynamic_bitset<> inputMask;

};

#endif /* FUZZYREASONER_H_ */
