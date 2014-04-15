%skeleton "lalr1.cc"
%debug
%defines
%define parser_class_name { FuzzyParser }
%define api.namespace {fz}
%define api.token.constructor
%define api.value.type variant
 
%code requires
{
	#include <sstream>
	#include <stdexcept>

	#include <vector>
	#include <map>
	#include <utility>
	#include "Node.h"
	
	class FuzzyBuilder;
	
	namespace fz
	{
		class FuzzyScanner;
	}
}

%parse-param { FuzzyBuilder  &builder  }
%parse-param { fz::FuzzyScanner  &scanner  }


%error-verbose
%locations
 
%code
{
	#include <fstream>
	
	#include "FuzzyBuilder.h"
	
	#undef yylex
	#define yylex scanner.lex
}

%token END 0

%token <std::string> ID
%token <std::string> VAR_ID
%token END_RULE
%token OP_OR
%token OP_AND
%token OP_NOT
%token OPEN_B
%token CLOSE_B
%token THEN
%token IS
%token IF

%token FUZZIFY
%token END_FUZZIFY
%token FUZZIFY_CLASS
%token END_FUZZIFY_CLASS
%token FUZZIFY_PREDICATE
%token END_FUZZIFY_PREDICATE

%token LIKE
%token COMMA
%token PERIOD
%token <std::string> F_LABEL
%token <int> PARAMETER

%type <Node*> wellFormedFormula
%type <Node*> fuzzyComparison
%type <Node*> fuzzyAssignment
%type <std::pair<std::string, std::string> > classMember
%type <std::vector<int>> shape
%type <std::vector<int>> parametersList
%type <std::vector<std::string> > fuzzyId
%type <std::string> var

%left OP_AND
%left OP_OR
%right OP_NOT

%%

fuzzyFile		: fuzzyClass fuzzySet ruleSet 
			;

fuzzyClass		: FUZZIFY_CLASS ID { builder.setNameSpace($2); } fuzzyPredicate END_FUZZIFY_CLASS fuzzyClass
			| /* Empty */
			;

fuzzyPredicate 		: FUZZIFY_PREDICATE ID fuzzyPredicateDef END_FUZZIFY_PREDICATE fuzzyPredicate
			| /* Empty */
			;

fuzzyPredicateDef 	: wellFormedFormula fuzzyAssignment END_RULE
			| /* Empty */
			;

fuzzySet		: FUZZIFY fuzzyId { builder.buildDomain($2); } fuzzyTerm END_FUZZIFY fuzzySet
			| /* Empty */
			;

fuzzyId			: ID 
			{
				$$.push_back($1);
			}
			| ID COMMA fuzzyId
			{
				$$ = $3;
				$$.push_back($1);
			}
			;

fuzzyTerm		: ID LIKE F_LABEL shape END_RULE
			{
				builder.buildMF($1, $3, $4);
			}
			| ID LIKE F_LABEL shape END_RULE fuzzyTerm 
			{
				builder.buildMF($1, $3, $4);
			}
			;

shape			: OPEN_B parametersList CLOSE_B 
			{
				$$ = $2;
			}
			;

parametersList		: PARAMETER 
			{ 
				$$.push_back($1);
			}
			| PARAMETER COMMA parametersList 
			{ 
				$$ = $3;
				$$.push_back($1);
			}
			;

ruleSet 		: rule ruleSet
			| /* Empty */
			;


rule 			: IF wellFormedFormula fuzzyAssignment END_RULE
			{
				builder.buildRule($2, $3);
			}
			;

wellFormedFormula	: fuzzyComparison 
			{
				$$ = $1;
			}
			| OPEN_B wellFormedFormula CLOSE_B 
			{
				$$ = $2;
			}
			| OP_NOT wellFormedFormula 
			{
				$$ = builder.buildNot($2);
			}
			| wellFormedFormula OP_OR wellFormedFormula 
			{
				$$ = builder.buildOr($1, $3);
			}
			| wellFormedFormula OP_AND wellFormedFormula 
			{
				$$ = builder.buildAnd($1,$3);
			}
			;

fuzzyComparison		: OPEN_B ID IS ID CLOSE_B 
			{
				$$ = builder.buildIs($2, $4);
			}
			| OPEN_B classMember IS ID CLOSE_B
			{
				$$ = builder.buildIs($2, $4);
			}
			;
			
fuzzyAssignment		: THEN OPEN_B ID IS ID CLOSE_B 
			{
				$$ = builder.buildAssignment($3, $5);
			}
			| THEN OPEN_B classMember IS ID CLOSE_B 
			{
				$$ = builder.buildAssignment($3, $5);
			}
			;

classMember		: ID PERIOD var
			{
				$$.first = $1;
				$$.second = $3;
			}
			;

var 			: ID
			{
				$$ = $1;
			}
			| VAR_ID
			{
				$$ = $1;
			}
			;
%%

void fz::FuzzyParser::error(const fz::FuzzyParser::location_type& l, const std::string& msg)
{
	std::stringstream ss;
	ss << "Error: " << msg << ", between " << l.begin << " and " << l.end << std::endl;
	throw std::runtime_error(ss.str());
}

