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
			ClassifierParam& params) :
			classification(classification),
			objects(classification.request.objects)
{
	classification.request.threshold = params.threshold;
	currentVars = NULL;
}

void ObjectClassificator::newObject(Feature& feature)
{
	size_t id = objects.size();
	objects.push_back(InputObject());

	InputObject& current = objects.back();
	current.id = id;
	currentVars = &current.variables;

	featureMap[id] = &feature;
}

void ObjectClassificator::addFeature(string name, int value)
{
	InputObject& current = objects.back();
	InputVariable feature;
	feature.name = name;
	feature.value = value;
	currentVars->push_back(feature);
}

void ObjectClassificator::addFeature(Feature& feature)
{
	newObject(feature);
	for (FeatureMap::const_iterator i = feature.begin(); i != feature.end();
				++i)
	{
		addFeature(i->first, i->second);
	}
}

void ObjectClassificator::labelFeatures()
{
	vector<ObjectClassification>& results = classification.response.results;

	for (vector<ObjectClassification>::iterator i = results.begin();
				i != results.end(); ++i)
	{
		Feature& feature = *featureMap[i->id];

		vector<ClassificationOutput>& outputs = i->classifications;

		for (vector<ClassificationOutput>::iterator j = outputs.begin();
					j != outputs.end(); ++j)
		{
			feature.addClassification(j->className, j->membership);
		}
	}

}

vector<pair<vector<cv::Point>, string> > ObjectClassificator::getGoodFeatures()
{
	vector<pair<vector<cv::Point>, string> > good;

	for (map<size_t, Feature*>::iterator it = featureMap.begin();
				it != featureMap.end(); ++it)
	{
		Feature& feature = *it->second;
		if (feature.isInteresting())
			good.push_back(
						std::make_pair(feature.getPointsVector(),
									feature.featureName()));
	}

	return good;
}

