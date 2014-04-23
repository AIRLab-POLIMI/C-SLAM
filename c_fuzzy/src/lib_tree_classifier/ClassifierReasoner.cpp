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

#include "ClassifierReasoner.h"

#include "RuleBuilder.h"
#include <iostream>

using namespace std;

ClassifierReasoner::ClassifierReasoner(FuzzyClassifier& classifier,
			FuzzyKnowledgeBase& knowledgeBase) :
			classifier(classifier), knowledgeBase(knowledgeBase)
{
	for (ClassList::iterator i = classifier.begin(); i != classifier.end(); ++i)
	{
		string className = i->first;
		FuzzyClass& fuzzyClass = *i->second;
		RuleBuilder builder(knowledgeBase);
		genVarTable[className] = builder.buildClassRule(fuzzyClass);
	}

	reasoner = new FuzzyReasoner(knowledgeBase);
}

void ClassifierReasoner::addInstance(ObjectInstance* instance)
{
	inputs.push_back(instance);
}

InstanceClassification ClassifierReasoner::run()
{
	InstanceClassification results;

	table.clear();

	for (ReasoningList::iterator i = classifier.beginReasoning();
				i != classifier.endReasoning(); ++i)
	{
		ClassList& classList = *i;
		map<string, ObjectList> candidates;

		getCandidates(classList, candidates);
		classify(classList, candidates, results);

	}

	return results;
}

void ClassifierReasoner::getCandidates(ClassList& classList,
			map<string, ObjectList>& candidates)
{
	for (ClassList::iterator it = classList.begin(); it != classList.end();
				++it)
	{
		string className = it->first;
		FuzzyClass* fuzzyClass = it->second;
		getClassCandidates(fuzzyClass, candidates[className]);
	}

}

void ClassifierReasoner::getClassCandidates(FuzzyClass* fuzzyClass,
			ObjectList& candidates)
{
	ObjectList& list = getSuperClassCandidates(fuzzyClass);

	for (ObjectList::iterator it = list.begin(); it != list.end(); ++it)
	{
		ObjectInstance* instance = *it;
		if (hasClassVariables(*instance, *fuzzyClass))
			candidates.push_back(instance);
	}
}

ObjectList& ClassifierReasoner::getSuperClassCandidates(FuzzyClass* fuzzyClass)
{
	FuzzyClass* superClass = fuzzyClass->getSuperClass();
	if (superClass)
	{
		return table[superClass->getName()];
	}
	else
	{
		return inputs;
	}
}

bool ClassifierReasoner::hasClassVariables(ObjectInstance& instance,
			FuzzyClass& fuzzyClass)
{
	const VariableList& variables = fuzzyClass.getVars();
	for (VariableList::const_iterator it = variables.begin();
				it != variables.end(); ++it)
	{
		if (instance.properties.count(*it) == 0)
		{
			return false;
		}
	}

	return true;
}

void ClassifierReasoner::classify(ClassList& classList,
			map<string, ObjectList>& candidates,
			InstanceClassification& results)
{
	vector<ObjectInstance*> instanceList;
	recursiveClassify(classList.begin(), classList.end(), candidates,
				instanceList, results);
}

void ClassifierReasoner::recursiveClassify(ClassList::iterator current, ClassList::iterator end,
			map<string, ObjectList>& candidates,
			vector<ObjectInstance*>& instanceList,
			InstanceClassification& results)
{

	if (current != end)
	{
		string currentClass = current->first;
		ObjectList& candidate = candidates[currentClass];
		current++;
		for (ObjectList::iterator it = candidate.begin(); it != candidate.end();
					++it)
		{
			instanceList.push_back(*it);
			recursiveClassify(current, end, candidates, instanceList, results);
			instanceList.pop_back();
		}

	}
	else
	{
		classifyInstances(instanceList, results);
	}
}

void ClassifierReasoner::classifyInstances(std::vector<ObjectInstance*>& instanceList,
				InstanceClassification& results)
{

}

