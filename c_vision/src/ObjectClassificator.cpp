/*
 * c_vision,
 *
 *
 * Copyright (C) 2014 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_vision.
 *
 * c_vision is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_vision is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_vision.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ObjectClassificator.h"

using namespace c_fuzzy;
using namespace std;

ObjectClassificator::ObjectClassificator(Classification& classification,
			double threshold) :
			classification(classification),
			objects(classification.request.objects)
{
	classification.request.threshold = threshold;
	currentVars = NULL;
}

void ObjectClassificator::newObject()
{
	size_t id = objects.size();
	InputObject current;
	current.id = id;
	currentVars = &current.variables;
	objects.push_back(current);
}

void ObjectClassificator::addFeature(string name, int value)
{
	InputObject& current = objects.back();
	InputVariable feature;
	feature.name = name;
	feature.value = value;
	currentVars->push_back(feature);
}

void ObjectClassificator::addFeature(const Feature& feature)
{
	newObject();
	for (FeatureMap::const_iterator i = feature.begin(); i != feature.end();
				++i)
	{
		addFeature(i->first, i->second);
	}
}

/*
ClassificationMap ObjectClassificator::getClassificationMap()
{
	ClassificationMap map;

	vector<ObjectClassification>& results = classification.response.results;
	for (vector<ObjectClassification>::iterator it = results.begin();
				it != results.end(); it++)
	{
		//TODO complete
	}

	return map;
}*/
