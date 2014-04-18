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

#ifndef FUZZYRULEENGINE_H_
#define FUZZYRULEENGINE_H_

#include <vector>
#include <string>

#include "FuzzyMF.h"
#include "VariableMasks.h"

class FuzzyVariableEngine
{
public:
	FuzzyVariableEngine()
	{
		initializeNamespaces();
		namespaceMasks = new NamespaceMasks();
	}

	void enterNamespace(std::string nameSpace);
	void addMF(std::string label, FuzzyMF* mf);
	void buildDomain(std::vector<std::string> variables);
	void updateVariableMask(std::pair<std::string, std::string>& variable,
				size_t currentRule);
	NamespaceTable* getTable();
	NamespaceMasks* getMasks();

private:
	void initializeNamespaces();

private:
	//Variable data
	NamespaceTable* namespaceTable;
	DomainTable* domainTable;
	MFTable* mfTable;

	//Mask data
	NamespaceMasks* namespaceMasks;

	//currentNamespace
	std::string currentNamespace;

};

#endif /* FUZZYRULEENGINE_H_ */
