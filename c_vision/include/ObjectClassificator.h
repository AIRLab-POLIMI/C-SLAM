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

#ifndef OBJECTCLASSIFICATOR_H_
#define OBJECTCLASSIFICATOR_H_

#include "Feature.h"

#include <c_fuzzy/Classification.h>
#include <string>
#include <vector>

class ObjectClassificator
{
public:
	ObjectClassificator(c_fuzzy::Classification& classification,
				double threshold);

	//ClassificationMap getClassificationMap();

	template<class T>
	void processFeatures(const std::vector<T>* features)
	{
		typedef typename std::vector<T> FeatureVector;

		for (typename FeatureVector::const_iterator i = features->begin();
					i != features->end(); ++i)
		{
			const Feature& feature = *i;
			addFeature(feature);
		}
	}

private:
	void newObject();
	void addFeature(std::string name, int value);
	void addFeature(const Feature& feature);

private:
	c_fuzzy::Classification& classification;
	std::vector<c_fuzzy::InputObject>& objects;
	std::vector<c_fuzzy::InputVariable>* currentVars;

};

#endif /* OBJECTCLASSIFICATOR_H_ */
