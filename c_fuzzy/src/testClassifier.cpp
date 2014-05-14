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

#include <map>
#include <stdexcept>

#include "FuzzyBuilder.h"
#include "TreeClassifierBuilder.h"
#include "ClassifierReasoner.h"

using namespace std;

void addInstances(ClassifierReasoner& reasoner)
{
	ObjectInstance* instance = new ObjectInstance();
	instance->id = 0;
	instance->properties["x"] = 100;
	instance->properties["y"] = 100;
	reasoner.addInstance(instance);

	instance = new ObjectInstance();
	instance->id = 1;
	instance->properties["x"] = 105;
	instance->properties["y"] = 105;
	reasoner.addInstance(instance);

	instance = new ObjectInstance();
	instance->id = 2;
	instance->properties["Xmin"] = 0;
	instance->properties["Xmax"] = 20;
	instance->properties["Ymin"] = 0;
	instance->properties["Ymax"] = 40;
	instance->properties["FormFactor"] = 1;
	reasoner.addInstance(instance);

	instance = new ObjectInstance();
	instance->id = 3;
	instance->properties["x"] = 18;
	instance->properties["y"] = 20;
	reasoner.addInstance(instance);
}

int main(int argc, char *argv[])
{

	try
	{
		FuzzyBuilder kbBuilder;
		TreeClassifierBuilder classifierBuilder;
		FuzzyClassifier* classifier;
		FuzzyKnowledgeBase* knowledgeBase;

		cout << "Parsing classifier file" << endl;
		classifierBuilder.parse(argv[1]);
		classifier = classifierBuilder.buildFuzzyClassifier();
		classifier->drawDependencyGraph("/home/dave/classifier.dot");
		classifier->drawReasoningGraph("/home/dave/reasoning.dot");
		cout << "Parsing done" << endl;

		cout << "Parsing knowledgeBase file" << endl;
		kbBuilder.parse(argv[2]);
		knowledgeBase = kbBuilder.createKnowledgeBase();
		cout << "Parsing done" << endl;

		cout << "Starting classifier reasoner" << endl;
		ClassifierReasoner reasoner(*classifier, *knowledgeBase);
		addInstances(reasoner);
		const InstanceClassification& classifications = reasoner.run(0);
		cout << "Reasoning done, results: " << endl;
		cout << classifications;

		delete classifier;
	}
	catch (const std::runtime_error& e)
	{
		cout << e.what() << endl;
		cout << "Check the input file an try again" << endl;
	}
}
