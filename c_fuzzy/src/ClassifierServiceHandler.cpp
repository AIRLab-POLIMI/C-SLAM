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
#include <sstream>
#include <ros/ros.h>
#include <ctime>

using namespace std;
using namespace c_fuzzy;

ClassifierServiceHandler::ClassifierServiceHandler(ros::NodeHandle& n,
			const string& knowledgeBasePath, const string& classifierPath)
{
	FuzzyBuilder kbBuilder;
	TreeClassifierBuilder classifierBuilder;

	classifierBuilder.parse(classifierPath.c_str());
	classifier = classifierBuilder.buildFuzzyClassifier();

	kbBuilder.parse(knowledgeBasePath.c_str());
	knowledgeBase = kbBuilder.createKnowledgeBase();

	reasoner = new ClassifierReasoner(*classifier, *knowledgeBase);

	classifierService = n.advertiseService("classification",
				&ClassifierServiceHandler::classificationCallback, this);

	rGraphService = n.advertiseService("getReasoningGraph",
				&ClassifierServiceHandler::reasoningGraphRequestCallback, this);

	dGraphService = n.advertiseService("getDependencyGraph",
				&ClassifierServiceHandler::dependencyGraphRequestCallback,
				this);
}

bool ClassifierServiceHandler::classificationCallback(
			Classification::Request& request,
			Classification::Response& response)
{
	ROS_DEBUG_STREAM(
				"Number of objects to classify: " << request.objects.size());
	clock_t begin = clock();
	vector<InputObject>& inputs = request.objects;
	vector<ObjectInstance> objects(inputs.size());

	addInputs(objects, inputs);
	const InstanceClassification& results = reasoner->run(request.threshold);
	sendOutputs(results, response);
	clock_t end = clock();
	double elapsed_ms = 1000 * double(end - begin) / CLOCKS_PER_SEC;
	ROS_DEBUG_STREAM("Service takes: " << elapsed_ms << "ms");

	return true;
}

bool ClassifierServiceHandler::reasoningGraphRequestCallback(
			c_fuzzy::Graph::Request& request,
			c_fuzzy::Graph::Response& response)
{
	stringstream ss;
	classifier->drawReasoningGraph(ss);
	response.graph = ss.str();
	return true;
}

bool ClassifierServiceHandler::dependencyGraphRequestCallback(
			c_fuzzy::Graph::Request& request,
			c_fuzzy::Graph::Response& response)
{
	stringstream ss;
	classifier->drawDependencyGraph(ss);
	response.graph = ss.str();
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
		for (ClassificationMap::const_iterator j = map.begin(); j != map.end();
					j++)
		{
			classification.classifications.push_back(ClassificationOutput());
			ClassificationOutput& out = classification.classifications.back();
			out.className = j->first;
			out.membership = j->second;
		}
	}
}

