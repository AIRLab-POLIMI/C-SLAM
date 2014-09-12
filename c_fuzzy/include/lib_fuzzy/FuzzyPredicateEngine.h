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

#ifndef FUZZYPREDICATEENGINE_H_
#define FUZZYPREDICATEENGINE_H_

#include <map>
#include <string>

#include "Node.h"
#include "FuzzyMF.h"

typedef std::pair<Node*, DomainTablePtr> PredicateInstance;

class FuzzyPredicateEngine
{

private:
	struct PredicateData
	{
		std::string templateVar;
		Node* definition;
	};

	typedef std::map<std::string, PredicateData> PredicateNameMap;
	typedef std::map<std::string, PredicateNameMap> PredicateMap;

public:
	FuzzyPredicateEngine();

	void enterNamespace(std::string nameSpace);
	void enterPredicate(std::string templateVariable);
	void buildDomain(std::string templateVar);
	void addTemplateMF(std::string label, FuzzyMF* mf);
	void buildPredicate(std::string name, Node* rule);
	PredicateInstance getPredicateInstance(std::string predicate,
			Variable variable);
	PredicateInstance getPredicateInstance(std::string nameSpace, std::string predicate,
			Variable variable);


private:
	DomainTablePtr instantiatePredicateVar(std::string nameSpace, std::string templateVar,
			std::string variable);

private:
	PredicateMap predicateMap;
	NamespaceTable table;
	std::string currentNamespace;
	std::string currentTemplateVar;

};

#endif /* FUZZYPREDICATEENGINE_H_ */
