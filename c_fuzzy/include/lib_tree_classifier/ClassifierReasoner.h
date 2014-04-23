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

#ifndef CLASSIFIERREASONER_H_
#define CLASSIFIERREASONER_H_

#include <map>
#include <vector>
#include <string>

#include "FuzzyKnowledgeBase.h"
#include "FuzzyClassifier.h"
#include "FuzzyReasoner.h"
#include "VariableGenerator.h"
#include "ClassificationData.h"

class ClassifierReasoner
{
private:
	typedef std::map<std::string, ObjectList> ClassificationTable;
public:
	ClassifierReasoner(FuzzyClassifier& classifier,
				FuzzyKnowledgeBase& knowledgeBase);
	void addInstance(ObjectInstance* instance);
	InstanceClassification run();

private:
	//instance retrival
	void getClassCandidates(FuzzyClass* fuzzyClass, ObjectList& candidates);
	ObjectList& getSuperClassCandidates(FuzzyClass* fuzzyClass);
	bool hasClassVariables(ObjectInstance& instance, FuzzyClass& fuzzyClass);
	void getCandidates(ClassList& classList,
				std::vector<ObjectList>& candidates);

	//classification
	void classify(ClassList& classList, std::vector<ObjectList>& candidates,
				InstanceClassification& results);

private:
	FuzzyClassifier& classifier;
	FuzzyKnowledgeBase& knowledgeBase;
	ObjectList inputs;
	FuzzyReasoner* reasoner;
	GeneratedVarTable genVarTable;

	ClassificationTable table;
};

#endif /* CLASSIFIERREASONER_H_ */
