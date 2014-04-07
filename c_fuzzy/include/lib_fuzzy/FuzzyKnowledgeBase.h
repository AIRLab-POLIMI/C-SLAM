/*
 * FuzzyKnowledgeBase.h
 *
 *  Created on: 21/mar/2014
 *      Author: dave
 */

#ifndef FUZZYKNOWLEDGEBASE_H_
#define FUZZYKNOWLEDGEBASE_H_

#include <map>
#include <string>
#include <vector>

#include "Node.h"
#include "VariableMasks.h"
#include "FuzzyMF.h"

class FuzzyKnowledgeBase
{
public:
	FuzzyKnowledgeBase(DomainTable* mfTable, VariableMasks* variableMasks,
				std::vector<Node*>* knowledgeBase) :
				domainTable(mfTable), variableMasks(variableMasks),
				knowledgeBase(knowledgeBase)
	{
	}
	size_t size();
	VariableMasks& getVariableMasks();
	DomainTable& getDomaintable();
	Node& operator[](size_t i);

	void addRule(Node* fuzzyRule, std::vector<std::string>& variables);

	~FuzzyKnowledgeBase();

private:
	void deleteMasks();
	void deleteRules();
	void deleteDomains();
	void deleteMF(MFTable* mfTable);
private:
	DomainTable* domainTable;
	VariableMasks* variableMasks;
	std::vector<Node*>* knowledgeBase;
};

#endif /* FUZZYKNOWLEDGEBASE_H_ */
