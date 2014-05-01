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
		ObjectListMap candidates;
		DepLists dependencies;

		getCandidates(classList, candidates, dependencies);
		classify(classList, dependencies, candidates, results);
	}

	return results;
}

void ClassifierReasoner::getCandidates(ClassList& classList,
			ObjectListMap& candidates, DepLists& deps)
{
	for (ClassList::iterator it = classList.begin(); it != classList.end();
				++it)
	{
		string className = it->first;
		FuzzyClass* fuzzyClass = it->second;
		getClassCandidates(fuzzyClass, candidates[className]);
		deps[className] = classifier.getDependenciesNames(fuzzyClass);
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

void ClassifierReasoner::classify(ClassList& classList, DepLists& deps,
			ObjectListMap& candidates, InstanceClassification& results)
{
	ClassificationData data(candidates, results);
	recursiveClassify(classList.begin(), classList.end(), deps, data);
}

void ClassifierReasoner::recursiveClassify(ClassList::iterator current,
			ClassList::iterator end, DepLists& deps, ClassificationData& data)
{

	if (current != end)
	{
		string currentClass = current->first;
		ObjectList& candidate = data.candidates[currentClass];
		current++;
		for (ObjectList::iterator it = candidate.begin(); it != candidate.end();
					++it)
		{
			ObjectInstance* instance = *it;
			if (hasBeenConsidered(instance, data))
				continue;
			data.instanceMap[currentClass] = instance;
			DepList& instanceDependencies = deps[currentClass];
			recursiveClassify(current, end, instanceDependencies.begin(),
						instanceDependencies.end(), deps, data);
			noMoreConsidered(instance, data);
		}

	}
	else
	{
		classifyInstances(data);
	}
}

void ClassifierReasoner::recursiveClassify(ClassList::iterator current,
			ClassList::iterator end, DepList::iterator currentDep,
			DepList::iterator endDep, DepLists& deps, ClassificationData& data)
{
	if (currentDep != endDep)
	{
		string dependencyName = *currentDep;
		currentDep++;
		ObjectList& dependencyObjects = getDependencyObjects(dependencyName, data);

		if (data.dependencyMap.count(dependencyName) == 0)
		{
			for (ObjectList::iterator it = dependencyObjects.begin();
						it != dependencyObjects.end(); ++it)
			{
				ObjectInstance* instance = *it;
				if (hasBeenConsidered(instance, data))
					continue;
				data.dependencyMap[dependencyName] = instance;
				recursiveClassify(current, end, currentDep, endDep, deps, data);
				noMoreConsidered(instance, data);
			}

			data.dependencyMap.erase(dependencyName);
		}
		else
		{
			recursiveClassify(current, end, currentDep, endDep, deps, data);
		}
	}
	else
	{
		recursiveClassify(current, end, deps, data);
	}
}

void ClassifierReasoner::classifyInstances(ClassificationData& data)
{
	setupReasoning(data);
	runReasoning(data);
}

void ClassifierReasoner::setupReasoning(ClassificationData& data)
{
	for (ObjectMap::iterator it = data.instanceMap.begin();
				it != data.instanceMap.end(); ++it)
	{
		string className = it->first;
		ObjectInstance* instance = it->second;
		ObjectProperties& properties = instance->properties;
		reasoner->addInput(className, properties);
		VariableGenerator* generator = genVarTable[className];
		ObjectProperties genProperties = generator->getGeneratedProperties(data.instanceMap, data.dependencyMap);
		reasoner->addInput(className, genProperties);
	}
}

ObjectList& ClassifierReasoner::getDependencyObjects(string& dependencyName, ClassificationData& data)
{
	if(data.candidates.count(dependencyName) == 1)
	{
		return data.candidates[dependencyName];
	}
	else
	{
		return table[dependencyName];
	}
}

void ClassifierReasoner::runReasoning(ClassificationData& data)
{
	OutputTable result = reasoner->run();
	for (ObjectMap::iterator i = data.instanceMap.begin();
				i != data.instanceMap.end(); ++i)
	{
		string className = i->first;
		ObjectInstance* instance = i->second;
		double truthValue = result[className][className].truth;
		ClassificationMap& instanceClassifications = data.results[instance->id];

		if (truthValue > 0
			&& (instanceClassifications.count(className) == 0
				|| instanceClassifications[className] < truthValue))
			instanceClassifications[className] = truthValue;

	}
}

bool ClassifierReasoner::hasBeenConsidered(ObjectInstance* instance,
			ClassificationData& data)
{
	TabuList& tabuList = data.tabuList;
	size_t id = instance->id;
	if (tabuList.count(id) != 0)
	{
		return true;
	}
	else
	{
		tabuList.insert(id);
		return false;
	}

}

void ClassifierReasoner::noMoreConsidered(ObjectInstance* instance,
			ClassificationData& data)
{
	TabuList& tabuList = data.tabuList;
	size_t id = instance->id;
	tabuList.erase(id);
}

