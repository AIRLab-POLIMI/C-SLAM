/*
 * FuzzyAttributes.h
 *
 *  Created on: 26/mar/2014
 *      Author: dave
 */

#ifndef FUZZYFEATURES_H_
#define FUZZYFEATURES_H_

#include <string>

class FuzzyFeature
{

};

class FuzzySimpleFeature: public FuzzyFeature
{
public:
	FuzzySimpleFeature(std::string variable, std::string fuzzyLabel) :
				variable(variable), fuzzyLabel(fuzzyLabel)
	{
	}

private:
	std::string variable;
	std::string fuzzyLabel;
};

class FuzzyRelation: public FuzzyFeature
{
public:
	FuzzyRelation(std::string className, std::string member) :
				className(className), member(member)
	{
	}

private:
	std::string className;
	std::string member;

};

class FuzzySimpleRelation: public FuzzyRelation
{
public:
	FuzzySimpleRelation(std::string className, std::string member,
				std::string matchingVar, std::string fuzzyLabel = "") :
				FuzzyRelation(className, member), matchingVar(matchingVar),
				fuzzyLabel(fuzzyLabel)
	{
	}

private:
	std::string matchingVar;
	std::string fuzzyLabel;
};

class FuzzyComplexRelation: public FuzzyRelation
{
public:
	FuzzyComplexRelation(std::string className, std::string member,
				std::string variable1, std::string variable2,
				std::string fuzzyLabel = "") :
				FuzzyRelation(className, member), variable1(variable1),
				variable2(variable2), fuzzyLabel(fuzzyLabel)
	{
	}

private:
	std::string variable1;
	std::string variable2;
	std::string fuzzyLabel;
};

typedef std::vector<FuzzyFeature*> FuzzyFeatureList;

#endif /* FUZZYFEATURES_H_ */
