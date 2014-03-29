/*
 * FuzzyAttributes.h
 *
 *  Created on: 26/mar/2014
 *      Author: dave
 */

#ifndef FUZZYFEATURES_H_
#define FUZZYFEATURES_H_

#include <string>
#include <vector>

enum FeatureType
{
	SIM_F, SIM_R, COM_R
};

typedef std::pair<std::vector<std::string>, FeatureType> FuzzyFeatureData;

class FuzzyFeature
{
public:
	virtual FeatureType getFeatureType() = 0;

	virtual std::vector<std::string> getVariables() = 0;

	virtual std::string getRelationObject() = 0;

	virtual std::string getRelationVariable() = 0;

	virtual ~FuzzyFeature()
	{
	}

};

class FuzzySimpleFeature: public FuzzyFeature
{
public:
	FuzzySimpleFeature(std::string variable, std::string fuzzyLabel) :
			variable(variable), fuzzyLabel(fuzzyLabel)
	{
	}

	virtual FeatureType getFeatureType()
	{
		return SIM_F;
	}

	virtual std::vector<std::string> getVariables()
	{
		std::vector<std::string> list;
		list.push_back(variable);
		return list;
	}

	virtual std::string getRelationObject()
	{
		return "";
	}

	virtual std::string getRelationVariable()
	{
		return "";
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

	virtual FeatureType getFeatureType() = 0;

	virtual std::string getRelationObject()
	{
		return className;
	}

	virtual std::string getRelationVariable()
	{
		return member;
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
			FuzzyRelation(className, member), matchingVar(matchingVar), fuzzyLabel(
					fuzzyLabel)
	{
	}

	virtual FeatureType getFeatureType()
	{
		return SIM_R;
	}

	virtual std::vector<std::string> getVariables()
	{
		std::vector<std::string> list;
		list.push_back(matchingVar);
		return list;
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
			FuzzyRelation(className, member), variable1(variable1), variable2(
					variable2), fuzzyLabel(fuzzyLabel)
	{
	}

	virtual FeatureType getFeatureType()
	{
		return COM_R;
	}

	virtual std::vector<std::string> getVariables()
	{
		std::vector<std::string> list;
		list.push_back(variable1);
		list.push_back(variable2);
		return list;
	}

private:
	std::string variable1;
	std::string variable2;
	std::string fuzzyLabel;
};

typedef std::vector<FuzzyFeature*> FuzzyFeatureList;

#endif /* FUZZYFEATURES_H_ */
