/*
 * FuzzyAttributes.h
 *
 *  Created on: 26/mar/2014
 *      Author: dave
 */

#ifndef FUZZYCONSTRAINTS_H_
#define FUZZYCONSTRAINTS_H_

#include <string>
#include <vector>

enum FeatureType
{
	SIM_C, SIM_R, COM_R, INV_R
};

typedef std::pair<std::vector<std::string>, FeatureType> FuzzyConstraintData;

class FuzzyConstraint
{
public:
	FuzzyConstraint(std::string fuzzyLabel) :
			fuzzyLabel(fuzzyLabel)
	{
	}

	std::string getFuzzyLabel()
	{
		return fuzzyLabel;
	}

	virtual FeatureType getConstraintType() = 0;

	virtual std::vector<std::string> getVariables() = 0;

	virtual std::string getRelationObject() = 0;

	virtual std::string getRelationVariable() = 0;

	virtual ~FuzzyConstraint()
	{
	}

private:
	std::string fuzzyLabel;

};

class FuzzySimpleConstraint: public FuzzyConstraint
{
public:
	FuzzySimpleConstraint(std::string variable, std::string fuzzyLabel) :
			FuzzyConstraint(fuzzyLabel), variable(variable)
	{
	}

	virtual FeatureType getConstraintType()
	{
		return SIM_C;
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
};

class FuzzyRelation: public FuzzyConstraint
{
public:
	FuzzyRelation(std::string className, std::string member,
			std::string fuzzyLabel) :
			FuzzyConstraint(fuzzyLabel), className(className), member(member)
	{
	}

	virtual FeatureType getConstraintType() = 0;

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
			std::string matchingVar, std::string fuzzyLabel) :
			FuzzyRelation(className, member, fuzzyLabel), matchingVar(
					matchingVar)
	{
	}

	virtual FeatureType getConstraintType()
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

};

class FuzzyComplexRelation: public FuzzyRelation
{
public:
	FuzzyComplexRelation(std::string className, std::string member,
			std::string variable1, std::string variable2,
			std::string fuzzyLabel) :
			FuzzyRelation(className, member, fuzzyLabel), variable1(variable1), variable2(
					variable2)
	{
	}

	virtual FeatureType getConstraintType()
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
};

class FuzzyInverseRelation: public FuzzyRelation
{
public:
	FuzzyInverseRelation(std::string className, std::string variable,
			std::string member1, std::string member2, std::string fuzzyLabel) :
			FuzzyRelation(className, variable, fuzzyLabel), member1(member1), member2(
					member2)
	{
	}

	virtual FeatureType getConstraintType()
	{
		return INV_R;
	}

	virtual std::vector<std::string> getVariables()
	{
		std::vector<std::string> list;
		list.push_back(member1);
		list.push_back(member2);
		return list;
	}

private:
	std::string member1;
	std::string member2;
};

typedef std::vector<FuzzyConstraint*> FuzzyConstraintsList;

#endif /* FUZZYCONSTRAINTS_H_ */
