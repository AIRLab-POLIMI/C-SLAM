%skeleton "lalr1.cc"
%debug
%defines
%define parser_class_name "FuzzyParser"
 
%code requires
{
	#include<vector>
	#include<map>
	#include "Node.h"
	class FuzzyBuilder;
	class FuzzyScanner;
}
 
%lex-param   { FuzzyScanner  &scanner  }
%parse-param { FuzzyScanner  &scanner  }
 
%lex-param   { FuzzyBuilder  &builder  }
%parse-param { FuzzyBuilder  &builder  }

%error-verbose
%locations
 
%code
{
	#include<iostream>
	#include<cstdlib>
	#include<fstream>
	
	#include "FuzzyBuilder.h"
	
	static int yylex(yy::FuzzyParser::semantic_type *yylval, yy::FuzzyParser::location_type* l, FuzzyScanner& scanner, FuzzyBuilder& builder);
}

%union {int integer; std::string* str; Node* node; std::vector<int>* fshape; std::vector<std::string>* fvars; }

%token<str> ID
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

%token LIKE
%token COMMA
%token <str> F_LABEL
%token <integer> PARAMETER

%type <node> wellFormedFormula
%type <node> fuzzyComparison
%type <node> fuzzyAssignment
%type <fshape> shape
%type <fshape> parametersList
%type <fvars> fuzzyId

%left OP_AND
%left OP_OR
%right OP_NOT

%%

fuzzyFile		: fuzzySet ruleSet 
			;

fuzzySet		: FUZZIFY fuzzyId { builder.buildDomain(*$2); delete $2; } fuzzyTerm END_FUZZIFY fuzzySet
			| /* Empty */
			;

fuzzyId			: ID 
			{
				$$ = new std::vector<std::string>;
				$$->push_back(*$1);
				delete $1;
			}
			| ID COMMA fuzzyId
			{
				$$ = $3;
				$$->push_back(*$1);
				delete $1;
			}
			;

fuzzyTerm		: ID LIKE F_LABEL shape END_RULE
			{
				builder.buildMF($1, $3, $4);
				delete $1;
				delete $3;
				delete $4;
			}
			| ID LIKE F_LABEL shape END_RULE fuzzyTerm 
			{
				builder.buildMF($1, $3, $4);
				delete $1;
				delete $3;
				delete $4;
			}
			;

shape			: OPEN_B parametersList CLOSE_B 
			{
				$$ = $2;
			}
			;

parametersList		: PARAMETER 
			{ 
				$$ = new std::vector<int>;
				$$->push_back($1);
			}
			| PARAMETER COMMA parametersList 
			{ 
				$$ = $3;
				$$->push_back($1);
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
				delete $2;
				delete $4;
			}
			;
			
fuzzyAssignment		: THEN OPEN_B ID IS ID CLOSE_B 
			{
				$$ = builder.buildAssignment($3, $5);
				delete $3;
				delete $5;
			}
			;
%%

void yy::FuzzyParser::error(const yy::FuzzyParser::location_type& l, const std::string& msg)
{
    std::cerr << "Error: " << msg << ", between " << l.begin << " and " << l.end << std::endl;
    exit(-1);
}

/* include for access to scanner.yylex */
#include "FuzzyScanner.h"
static int yylex(yy::FuzzyParser::semantic_type *yylval, yy::FuzzyParser::location_type* l, FuzzyScanner& scanner, FuzzyBuilder& builder)
{
	l->step();
	return(scanner.yylex(yylval));
}

