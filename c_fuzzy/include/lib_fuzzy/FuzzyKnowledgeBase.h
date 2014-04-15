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
	FuzzyKnowledgeBase(NamespaceTable* namespaceTable,
			NamespaceMasks* namespaceMasks, std::vector<Node*>* knowledgeBase) :
			namespaceTable(namespaceTable), namespaceMasks(namespaceMasks), knowledgeBase(
					knowledgeBase)
	{
	}
	size_t size();
	NamespaceMasks& getNamespaceMasks();
	NamespaceTable& getNamespaceTable();
	Node& operator[](size_t i);

	void addRule(Node* fuzzyRule,
			std::vector<std::pair<std::string, std::string> >& variables);

	~FuzzyKnowledgeBase();

private:
	void deleteMasks();
	void deleteRules();
	void deleteDomains();
	void deleteMF(MFTable* mfTable);

private:
	NamespaceTable* namespaceTable;
	NamespaceMasks* namespaceMasks;
	std::vector<Node*>* knowledgeBase;
};

#endif /* FUZZYKNOWLEDGEBASE_H_ */
