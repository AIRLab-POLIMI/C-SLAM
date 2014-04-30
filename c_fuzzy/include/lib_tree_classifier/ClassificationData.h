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

#ifndef CLASSIFICATIONDATA_H_
#define CLASSIFICATIONDATA_H_

#include <string>
#include <vector>
#include <map>
#include <set>
#include <iostream>

typedef std::map<std::string, int> ObjectProperties;
typedef std::map<std::string, double> ClassificationMap;
typedef std::map<size_t, ClassificationMap> InstanceClassification;

inline std::ostream& operator<<(std::ostream& os,
			const InstanceClassification& inputs)
{
	for (InstanceClassification::const_iterator i = inputs.begin();
				i != inputs.end(); ++i)
	{
		for (ClassificationMap::const_iterator j = i->second.begin();
					j != i->second.end(); ++j)
		{
			os << "Instance #" << i->first << std::endl;
			os << j->first << "=" << j->second << std::endl;
		}
	}

	return os;
}

struct ObjectInstance
{
	size_t id;
	ObjectProperties properties;
};

typedef std::vector<ObjectInstance*> ObjectList;
typedef std::map<std::string, ObjectList> ObjectListMap;
typedef std::map<std::string, ObjectInstance*> ObjectMap;

typedef std::set<size_t> TabuList;

struct ClassificationData
{
	ClassificationData(ObjectListMap& candidates,
				InstanceClassification& results) :
				candidates(candidates), results(results)
	{
	}

	//local classification data
	ObjectMap instanceMap;
	ObjectMap dependencyMap;
	TabuList tabuList;

	//Global classification data
	ObjectListMap& candidates;
	InstanceClassification& results;
};

#endif /* CLASSIFICATIONDATA_H_ */
