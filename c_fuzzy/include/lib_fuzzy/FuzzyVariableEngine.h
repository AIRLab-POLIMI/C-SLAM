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
	FuzzyVariableEngine();

	void enterNamespace(std::string nameSpace);
	void addMF(std::string label, FuzzyMF* mf);
	void addDomains(std::string nameSpace, DomainTable* domain);
	void buildDomain(std::vector<std::string> variables);
	void normalizeVariableMasks(size_t size);
	void updateVariableMask(Variable& var, size_t rule);
	void updateVariableMask(
			std::vector<Variable>& vars,
			size_t rule);
	NamespaceTable& getTable();
	VariableMasks& getMasks();
	~FuzzyVariableEngine();

private:
	void initializeNamespaces();
	void deleteMasks();
	void deleteDomains();
	void deleteMF(MFTable* mfTable);

private:
	//Variable data
	NamespaceTable namespaceTable;
	DomainTable* domainTable;
	MFTable* mfTable;

	//Mask data
	VariableMasks variableMasks;

	//currentNamespace
	std::string currentNamespace;

};

#endif /* FUZZYRULEENGINE_H_ */
