%skeleton "lalr1.cc"
%debug
%defines
%define parser_class_name { TreeClassifierParser }
%define api.namespace {tc}
%define api.token.constructor
%define api.value.type variant
 
%code requires
{
	#include <sstream>
	#include <stdexcept>

	#include <string>
	#include <vector>
	#include "FuzzyClass.h" 

	class TreeClassifierBuilder;
	namespace tc
	{
		class TreeClassifierScanner;
	}
}
 
%parse-param { TreeClassifierBuilder  &builder  }
%parse-param { tc::TreeClassifierScanner  &scanner  }

%error-verbose
%locations
 
%code
{
	#include<fstream>

	#include "TreeClassifierBuilder.h"
	#include "TreeClassifierParser.tab.h"
	#include "TreeClassifierScanner.h"

	
	#undef yylex
	#define yylex scanner.lex
	
}

%token END 0

%token <std::string> ID VAR_ID
%token IS
%token MATCH
%token ON
%token DEGREE

%token CLASS
%token VARIABLES
%token END_VARIABLES
%token CONSTANTS
%token END_CONSTANTS
%token END_CLASS
%token EXTENDS
%token IMPORTANT

%token PERIOD
%token SEMICOLON
%token COMMA
%token LPAR
%token RPAR
%token EQUAL

%type <bool> importantFlag
%type <std::string> var
%type <std::string> fuzzySuperclass fuzzyConstraint fuzzyDegree
%type <FuzzyFeatureData> fuzzyFeature;
%type <std::vector<std::string> > fuzzySimpleFeature fuzzySimpleRelation fuzzyComplexRelation fuzzyInverseRelation
%type <VariableList*> variables variableList
%type <ConstantList*> constants constantList
%type <ElementsList> fuzzyClassElements
%type <FuzzyFeatureList*> fuzzyFeatures
 

%%

fuzzyClassifiers	: fuzzyClass fuzzyClassifiers
			| fuzzyClass
			;

fuzzyClass		: CLASS ID fuzzySuperclass importantFlag fuzzyClassElements fuzzyFeatures END_CLASS
			{
				builder.buildClass($2, $3, $5.first, $5.second, $6, $4);
			}
			;

fuzzySuperclass		: EXTENDS ID 
			{
				$$ = $2;
			}
			| /* empty */
			{
				$$ = "";
			}
			;

importantFlag		: IMPORTANT 
			{
				$$ = true;
			}
			| /* empty */
			{
				$$ = false;
			}
			;

fuzzyClassElements	: constants variables
			{
				$$.first = $2;
				$$.second = $1;
			}
			| variables constants
			{
				$$.first = $1;
				$$.second = $2;
			}
			| constants
			{
				$$.first = NULL;
				$$.second = $1;
			}
			| variables
			{
				$$.first = $1;
				$$.second = NULL;
			}
			| /* empty */
			{
				$$.first = NULL;
				$$.second = NULL;
			}
			;

constants		: CONSTANTS constantList END_CONSTANTS
			{
				$$ = $2;
			}
			;
			
constantList		: var EQUAL ID SEMICOLON constantList
			{
				$$ = builder.buildCostantList($5, $1, $3);

			}
			| /* empty */
			{
				$$ = NULL;
			}
			;

variables		: VARIABLES variableList END_VARIABLES
			{
				$$ = $2;
			}
			;

variableList		: var SEMICOLON variableList
			{
				$$ = builder.buildVariableList($3, $1);
			}
			| /* empty */
			{
				$$ = NULL;
			}
			;


fuzzyFeatures		: fuzzyFeature SEMICOLON fuzzyFeatures
			{
				$$ = builder.buildFeaturesList($3, $1.first, $1.second);
			}
			| /* empty */
			{
				$$ = NULL;
			}
			;

fuzzyFeature		: fuzzySimpleFeature
			{
				$$.first = $1;
				$$.second = SIM_F;
			}
			| fuzzySimpleRelation
			{
				$$.first = $1;
				$$.second = SIM_R;
			}
			| fuzzyComplexRelation
			{
				$$.first = $1;
				$$.second = COM_R;
			}
			| fuzzyInverseRelation
			{
				$$.first = $1;
				$$.second = INV_R;
			}
			;

fuzzySimpleFeature	: var IS ID
			{
				$$.push_back($1);
				$$.push_back($3);
			}
			;

fuzzySimpleRelation	: ID PERIOD var MATCH var fuzzyDegree
			{
				$$.push_back($1);
				$$.push_back($3);
				$$.push_back($5);
				$$.push_back($6);
			}
			;

fuzzyComplexRelation	: ID PERIOD var fuzzyConstraint ON LPAR var COMMA var RPAR
			{
				$$.push_back($1);
				$$.push_back($3);
				$$.push_back($7);
				$$.push_back($9);
				$$.push_back($4);
			}
			;
			
fuzzyInverseRelation	: var fuzzyConstraint ON ID LPAR var COMMA var RPAR
			{
				$$.push_back($1);
				$$.push_back($4);
				$$.push_back($6);
				$$.push_back($8);
				$$.push_back($2);
			}
			;


fuzzyConstraint		: IS ID
			{
				$$ = $2;
			}
			| /* empty */
			{
				$$ = "";
			}
			;
			
fuzzyDegree		: DEGREE ID
			{
				$$ = $2;
			}
			| /* empty */
			{
				$$ = "";
			}
			;


var			: ID
			{
				$$ = $1;
			}
			| VAR_ID
			{
				$$ = $1;
			}
			;

%%

void tc::TreeClassifierParser::error(const tc::TreeClassifierParser::location_type& l, const std::string& msg)
{
	std::stringstream ss;
	ss << "Error: " << msg << ", between " << l.begin << " and " << l.end << std::endl;
	throw std::runtime_error(ss.str());
}

