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

#ifndef CLASSIFIERSERVICEHANDLER_H_
#define CLASSIFIERSERVICEHANDLER_H_

#include <vector>
#include <string>

#include "ClassifierReasoner.h"

#include "c_fuzzy/Classification.h"
#include "c_fuzzy/Graph.h"

class ClassifierServiceHandler
{
public:
	ClassifierServiceHandler(const std::string& knowledgeBasePath,
				const std::string& classifierPath);

	bool classificationCallback(c_fuzzy::Classification::Request& request,
				c_fuzzy::Classification::Response& response);

	bool reasoningGraphRequestCallback(c_fuzzy::Graph::Request& request,
				c_fuzzy::Graph::Response& response);
	bool dependencyGraphRequestCallback(c_fuzzy::Graph::Request& request,
				c_fuzzy::Graph::Response& response);
	~ClassifierServiceHandler();

private:
	FuzzyKnowledgeBase* knowledgeBase;
	FuzzyClassifier* classifier;
	ClassifierReasoner* reasoner;

	void addInputs(std::vector<ObjectInstance>& objects,
				std::vector<c_fuzzy::InputObject>& inputs);
	void sendOutputs(const InstanceClassification& results,
				c_fuzzy::Classification::Response& response);
};

#endif /* CLASSIFIERSERVICEHANDLER_H_ */
