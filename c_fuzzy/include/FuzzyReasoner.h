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

#include<map>
#include<string>
#include<vector>

#include "Node.h"
#include "FuzzyMF.h"
#include "FuzzyAggregator.h"

/**
 * The class implementing the reasoner.
 *
 *
 */
class FuzzyReasoner
{
public:
	FuzzyReasoner(std::map<std::string, int>* inputTable,
			std::map<std::string, FuzzyMF*>* mfTable,
			FuzzyAggregator* aggregator, std::vector<Node*>* knowledgeBase) :
			inputTable(inputTable), mfTable(mfTable), aggregator(aggregator), knowledgeBase(
					knowledgeBase)
	{
	}
	void addRule(Node* fuzzyRule);
	void addInput(std::string name, int value);
	std::map<std::string, double> run();
	~FuzzyReasoner();

private:
	void deleteMF();
	void deleteRules();

private:
	std::map<std::string, int>* inputTable;
	std::map<std::string, FuzzyMF*>* mfTable;
	FuzzyAggregator* aggregator;
	std::vector<Node*>* knowledgeBase;
};

#endif /* FUZZYREASONER_H_ */
