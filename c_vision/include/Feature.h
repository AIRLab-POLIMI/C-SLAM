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

#ifndef FEATURE_H_
#define FEATURE_H_

#include <string>
#include <map>

#include <opencv2/features2d/features2d.hpp>

/**
 * Base class  of feature that can be detected
 */

typedef std::map<std::string, int> FeatureMap;
typedef std::map<std::string, double> ClassificationMap;

class Feature
{
public:

	inline FeatureMap::const_iterator begin() const
	{
		return featureMap.begin();
	}

	inline FeatureMap::const_iterator end() const
	{
		return featureMap.end();
	}

	inline void addClassification(std::string classification, double truthValue)
	{
		classifications[classification] = truthValue;
	}

	inline ClassificationMap::const_iterator beginClass() const
	{
		return classifications.begin();
	}

	inline ClassificationMap::const_iterator endClass() const
	{
		return classifications.end();
	}

	virtual void setFeature() = 0;

	virtual ~Feature()
	{
	}

protected:
	FeatureMap featureMap;
	ClassificationMap classifications;

};

#endif /* FEATURE_H_ */
