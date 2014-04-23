/*
 * c_fuzzy,
 *
 *
 * Copyright (C) 2013 Davide Tateo
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

#ifndef REASONINGDATA_H_
#define REASONINGDATA_H_

#include <map>
#include <string>
#include <ostream>

/**
 * Helper struct for passing data about fuzzy labels
 */
struct FuzzyData
{
	double weight;
	double value;
	int cardinality;
};

/**
 * Type used to store labels computed
 */
typedef std::map<std::string, FuzzyData> DataMap;

/**
 * Type used to store aggregation results
 */
typedef std::map<std::string, DataMap> DomainAggregationMap;
typedef std::map<std::string, DomainAggregationMap> AggregationMap;

/**
 * Fuzzy aggregation operator
 * the aggregation operation used is the average
 *
 */
class FuzzyAggregator
{
public:
	void addValue(std::string nameSpace, std::string output,
			std::string mfLabel, double weight, double value);
	AggregationMap getAggregations();

private:
	DataMap getAggregation(DataMap outputs);
	DataMap createDataMap(std::string mfLabel, double value, double weight);
	FuzzyData createData(double value, double weight);

private:
	AggregationMap aggregationMap;

};

/**
 * Input map type for collecting all the inputs
 */
typedef std::map<std::string, std::map<std::string, int> > InputTable;

inline std::ostream& operator<<(std::ostream& os, const InputTable& inputs)
{
	for (InputTable::const_iterator i = inputs.begin(); i != inputs.end(); ++i)
	{
		for (std::map<std::string, int>::const_iterator j = i->second.begin();
				j != i->second.end(); ++j)
		{
			if(!i->first.empty())
			{
				os << i->first << ".";
			}

			os << j->first << "=" << j->second
					<< std::endl;

		}
	}

	return os;
}

/**
 * Reasoning data struct
 */
struct ReasoningData
{
	ReasoningData(InputTable& inputs, FuzzyAggregator& aggregator) :
			inputs(inputs), aggregator(aggregator)
	{
		truthValue = 0;
		inputValue = 0;
	}
	InputTable& inputs;
	FuzzyAggregator& aggregator;
	double truthValue;
	int inputValue;
};

/**
 * This struct is used to store the couple value/truth level
 *
 */
struct FuzzyOutput
{
	double value;
	double truth;
};

/*
 * Type to store the corresponding domain outputs
 */
typedef std::map<std::string, FuzzyOutput> DomainOutputTable;

/*
 * Type to store the corresponding namespace outputs
 */
typedef std::map<std::string, DomainOutputTable> OutputTable;

inline std::ostream& operator<<(std::ostream& os, const OutputTable& outputs)
{
	for (OutputTable::const_iterator i = outputs.begin(); i != outputs.end(); ++i)
	{
		const DomainOutputTable& map = i->second;
		for (DomainOutputTable::const_iterator j = map.begin(); j != map.end(); ++j)
		{

			if (!i->first.empty())
			{
				os << i->first << ".";
			}

			os << j->first << " = " << j->second.value
					<< ", truth value: = " << j->second.truth << std::endl;
		}
	}

	return os;
}


#endif /* REASONINGDATA_H_ */
