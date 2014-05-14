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

#include <vector>
#include <map>
#include <string>

#include "ClassifierServiceHandler.h"
#include "FuzzyBuilder.h"
#include "TreeClassifierBuilder.h"

#include <iostream>

using namespace std;
using namespace c_fuzzy;

ClassifierServiceHandler::ClassifierServiceHandler(
			const char* knowledgeBasePath, const char* classifierPath)
{
	FuzzyBuilder kbBuilder;
	TreeClassifierBuilder classifierBuilder;

	classifierBuilder.parse(classifierPath);
	classifier = classifierBuilder.buildFuzzyClassifier();

	kbBuilder.parse(knowledgeBasePath);
	knowledgeBase = kbBuilder.createKnowledgeBase();

	reasoner = new ClassifierReasoner(*classifier, *knowledgeBase);
}

bool ClassifierServiceHandler::classificationCallback(
			Classification::Request& request,
			Classification::Response& response)
{
	vector<InputObject>& inputs = request.objects;
	vector<ObjectInstance> objects(inputs.size());

	addInputs(objects, inputs);
	const InstanceClassification& results = reasoner->run(request.threshold);
	sendOutputs(results, response);

	return true;
}

ClassifierServiceHandler::~ClassifierServiceHandler()
{
	delete reasoner;
	delete classifier;
	delete knowledgeBase;
}

void ClassifierServiceHandler::addInputs(vector<ObjectInstance>& objects,
			vector<InputObject>& inputs)
{
	for (size_t i = 0; i < inputs.size(); i++)
	{
		InputObject& input = inputs[i];
		ObjectInstance& instance = objects[i];
		instance.id = input.id;
		vector<InputVariable>& map = input.variables;
		for (vector<InputVariable>::iterator j = map.begin(); j != map.end();
					++j)
		{
			instance.properties[j->name] = j->value;
		}


		reasoner->addInstance(&instance);
	}
}

void ClassifierServiceHandler::sendOutputs(
			const InstanceClassification& results,
			Classification::Response& response)
{
	for (InstanceClassification::const_iterator it = results.begin();
				it != results.end(); ++it)
	{
		response.results.push_back(ObjectClassification());
		ObjectClassification& classification = response.results.back();
		classification.id = it->first;
		const ClassificationMap& map = it->second;
		for (ClassificationMap::const_iterator j = map.begin(); j != map.end(); j++)
		{
			classification.classifications.push_back(ClassificationOutput());
			ClassificationOutput& out = classification.classifications.back();
			out.className = j->first;
			out.membership = j->second;
		}
	}
}

