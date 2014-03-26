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

#include <vector>
#include <map>
#include <string>

#include "ReasonerServiceHandler.h"

#include "FuzzyBuilder.h"

#include "c_fuzzy/InputVariable.h"
#include "c_fuzzy/DefuzzyfiedOutput.h"

#include <iostream>

using namespace std;
using namespace c_fuzzy;

ReasonerServiceHandler::ReasonerServiceHandler(const char* knowledgeBasePath)
{
	FuzzyBuilder builder;

	builder.parse(knowledgeBasePath);

	knowledgeBase = builder.createKnowledgeBase();
}

bool ReasonerServiceHandler::reasoningCallback(Reasoning::Request& request,
			Reasoning::Response& response)
{
	FuzzyReasoner reasoner(*knowledgeBase);
	vector<c_fuzzy::InputVariable>::iterator i;

	for (i = request.inputs.begin(); i != request.inputs.end(); ++i)
	{
		reasoner.addInput(i->name, i->value);
	}

	map<string, FuzzyOutput> results = reasoner.run();

	map<string, FuzzyOutput>::iterator j;

	for (j = results.begin(); j != results.end(); ++j)
	{
		DefuzzyfiedOutput output;
		output.name = j->first;
		output.value = j->second.value;
		output.truth = j->second.truth;
		response.results.push_back(output);
	}

	return true;
}

ReasonerServiceHandler::~ReasonerServiceHandler()
{
	delete knowledgeBase;
}
