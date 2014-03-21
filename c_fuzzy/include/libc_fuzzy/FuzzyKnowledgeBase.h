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
#include "BitData.h"
#include "FuzzyMF.h"


class FuzzyKnowledgeBase
{
public:
	FuzzyKnowledgeBase(DomainTable* mfTable,
			std::map<std::string, BitData>* variableMasks,
			std::vector<Node*>* knowledgeBase) :
			domainTable(mfTable), variableMasks(variableMasks), knowledgeBase(
					knowledgeBase)
	{
	}
	unsigned long size();
	void addRule(Node* fuzzyRule);
	std::map<std::string, BitData>& getVariableMasks();
	~FuzzyKnowledgeBase();

public:
	Node& operator[](size_t i);
private:
	void deleteMasks();
	void deleteRules();
	void deleteDomains();
	void deleteMF(MFTable* mfTable);
private:
	DomainTable* domainTable;
	std::map<std::string, BitData>* variableMasks;
	std::vector<Node*>* knowledgeBase;
};

#endif /* FUZZYKNOWLEDGEBASE_H_ */
