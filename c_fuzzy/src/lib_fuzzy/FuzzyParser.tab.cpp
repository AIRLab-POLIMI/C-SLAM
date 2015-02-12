// A Bison parser, made by GNU Bison 3.0.2.

// Skeleton implementation for Bison LALR(1) parsers in C++

// Copyright (C) 2002-2013 Free Software Foundation, Inc.

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// As a special exception, you may create a larger work that contains
// part or all of the Bison parser skeleton and distribute that work
// under terms of your choice, so long as that work isn't itself a
// parser generator using the skeleton or a modified version thereof
// as a parser skeleton.  Alternatively, if you modify or redistribute
// the parser skeleton itself, you may (at your option) remove this
// special exception, which will cause the skeleton and the resulting
// Bison output files to be licensed under the GNU General Public
// License without this special exception.

// This special exception was added by the Free Software Foundation in
// version 2.2 of Bison.


// First part of user declarations.

#line 37 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:399

# ifndef YY_NULLPTR
#  if defined __cplusplus && 201103L <= __cplusplus
#   define YY_NULLPTR nullptr
#  else
#   define YY_NULLPTR 0
#  endif
# endif

#include "FuzzyParser.tab.h"

// User implementation prologue.

#line 51 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:407
// Unqualified %code blocks.
#line 35 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:408

	#include <fstream>
	
	#include "FuzzyBuilder.h"
	#include "FuzzyScanner.h"
	#include "FuzzyParser.tab.h"
	
	#undef yylex
	#define yylex scanner.lex

#line 64 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:408


#ifndef YY_
# if defined YYENABLE_NLS && YYENABLE_NLS
#  if ENABLE_NLS
#   include <libintl.h> // FIXME: INFRINGES ON USER NAME SPACE.
#   define YY_(msgid) dgettext ("bison-runtime", msgid)
#  endif
# endif
# ifndef YY_
#  define YY_(msgid) msgid
# endif
#endif

#define YYRHSLOC(Rhs, K) ((Rhs)[K].location)
/* YYLLOC_DEFAULT -- Set CURRENT to span from RHS[1] to RHS[N].
   If N is 0, then set CURRENT to the empty location which ends
   the previous symbol: RHS[0] (always defined).  */

# ifndef YYLLOC_DEFAULT
#  define YYLLOC_DEFAULT(Current, Rhs, N)                               \
    do                                                                  \
      if (N)                                                            \
        {                                                               \
          (Current).begin  = YYRHSLOC (Rhs, 1).begin;                   \
          (Current).end    = YYRHSLOC (Rhs, N).end;                     \
        }                                                               \
      else                                                              \
        {                                                               \
          (Current).begin = (Current).end = YYRHSLOC (Rhs, 0).end;      \
        }                                                               \
    while (/*CONSTCOND*/ false)
# endif


// Suppress unused-variable warnings by "using" E.
#define YYUSE(E) ((void) (E))

// Enable debugging if requested.
#if YYDEBUG

// A pseudo ostream that takes yydebug_ into account.
# define YYCDEBUG if (yydebug_) (*yycdebug_)

# define YY_SYMBOL_PRINT(Title, Symbol)         \
  do {                                          \
    if (yydebug_)                               \
    {                                           \
      *yycdebug_ << Title << ' ';               \
      yy_print_ (*yycdebug_, Symbol);           \
      *yycdebug_ << std::endl;                  \
    }                                           \
  } while (false)

# define YY_REDUCE_PRINT(Rule)          \
  do {                                  \
    if (yydebug_)                       \
      yy_reduce_print_ (Rule);          \
  } while (false)

# define YY_STACK_PRINT()               \
  do {                                  \
    if (yydebug_)                       \
      yystack_print_ ();                \
  } while (false)

#else // !YYDEBUG

# define YYCDEBUG if (false) std::cerr
# define YY_SYMBOL_PRINT(Title, Symbol)  YYUSE(Symbol)
# define YY_REDUCE_PRINT(Rule)           static_cast<void>(0)
# define YY_STACK_PRINT()                static_cast<void>(0)

#endif // !YYDEBUG

#define yyerrok         (yyerrstatus_ = 0)
#define yyclearin       (yyempty = true)

#define YYACCEPT        goto yyacceptlab
#define YYABORT         goto yyabortlab
#define YYERROR         goto yyerrorlab
#define YYRECOVERING()  (!!yyerrstatus_)

#line 5 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:474
namespace fz {
#line 150 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:474

  /* Return YYSTR after stripping away unnecessary quotes and
     backslashes, so that it's suitable for yyerror.  The heuristic is
     that double-quoting is unnecessary unless the string contains an
     apostrophe, a comma, or backslash (other than backslash-backslash).
     YYSTR is taken from yytname.  */
  std::string
   FuzzyParser ::yytnamerr_ (const char *yystr)
  {
    if (*yystr == '"')
      {
        std::string yyr = "";
        char const *yyp = yystr;

        for (;;)
          switch (*++yyp)
            {
            case '\'':
            case ',':
              goto do_not_strip_quotes;

            case '\\':
              if (*++yyp != '\\')
                goto do_not_strip_quotes;
              // Fall through.
            default:
              yyr += *yyp;
              break;

            case '"':
              return yyr;
            }
      do_not_strip_quotes: ;
      }

    return yystr;
  }


  /// Build a parser object.
   FuzzyParser :: FuzzyParser  (FuzzyBuilder  &builder_yyarg, fz::FuzzyScanner  &scanner_yyarg)
    :
#if YYDEBUG
      yydebug_ (false),
      yycdebug_ (&std::cerr),
#endif
      builder (builder_yyarg),
      scanner (scanner_yyarg)
  {}

   FuzzyParser ::~ FuzzyParser  ()
  {}


  /*---------------.
  | Symbol types.  |
  `---------------*/



  // by_state.
  inline
   FuzzyParser ::by_state::by_state ()
    : state (empty)
  {}

  inline
   FuzzyParser ::by_state::by_state (const by_state& other)
    : state (other.state)
  {}

  inline
  void
   FuzzyParser ::by_state::move (by_state& that)
  {
    state = that.state;
    that.state = empty;
  }

  inline
   FuzzyParser ::by_state::by_state (state_type s)
    : state (s)
  {}

  inline
   FuzzyParser ::symbol_number_type
   FuzzyParser ::by_state::type_get () const
  {
    return state == empty ? 0 : yystos_[state];
  }

  inline
   FuzzyParser ::stack_symbol_type::stack_symbol_type ()
  {}


  inline
   FuzzyParser ::stack_symbol_type::stack_symbol_type (state_type s, symbol_type& that)
    : super_type (s, that.location)
  {
      switch (that.type_get ())
    {
      case 47: // wellFormedFormula
      case 48: // fuzzyComparison
      case 49: // fuzzyPredicateCall
      case 50: // fuzzyAssignment
        value.move< NodePtr > (that.value);
        break;

      case 51: // variable
        value.move< Variable > (that.value);
        break;

      case 25: // PARAMETER
        value.move< int > (that.value);
        break;

      case 3: // ID
      case 4: // VAR_ID
      case 24: // F_LABEL
      case 54: // var
      case 55: // templateVar
        value.move< std::string > (that.value);
        break;

      case 52: // variableList
      case 53: // variableListHelper
        value.move< std::vector<Variable> > (that.value);
        break;

      case 43: // shape
      case 44: // parametersList
        value.move< std::vector<int> > (that.value);
        break;

      case 41: // fuzzyId
      case 56: // templateVarList
      case 57: // templateVarListHelper
        value.move< std::vector<std::string> > (that.value);
        break;

      default:
        break;
    }

    // that is emptied.
    that.type = empty;
  }

  inline
   FuzzyParser ::stack_symbol_type&
   FuzzyParser ::stack_symbol_type::operator= (const stack_symbol_type& that)
  {
    state = that.state;
      switch (that.type_get ())
    {
      case 47: // wellFormedFormula
      case 48: // fuzzyComparison
      case 49: // fuzzyPredicateCall
      case 50: // fuzzyAssignment
        value.copy< NodePtr > (that.value);
        break;

      case 51: // variable
        value.copy< Variable > (that.value);
        break;

      case 25: // PARAMETER
        value.copy< int > (that.value);
        break;

      case 3: // ID
      case 4: // VAR_ID
      case 24: // F_LABEL
      case 54: // var
      case 55: // templateVar
        value.copy< std::string > (that.value);
        break;

      case 52: // variableList
      case 53: // variableListHelper
        value.copy< std::vector<Variable> > (that.value);
        break;

      case 43: // shape
      case 44: // parametersList
        value.copy< std::vector<int> > (that.value);
        break;

      case 41: // fuzzyId
      case 56: // templateVarList
      case 57: // templateVarListHelper
        value.copy< std::vector<std::string> > (that.value);
        break;

      default:
        break;
    }

    location = that.location;
    return *this;
  }


  template <typename Base>
  inline
  void
   FuzzyParser ::yy_destroy_ (const char* yymsg, basic_symbol<Base>& yysym) const
  {
    if (yymsg)
      YY_SYMBOL_PRINT (yymsg, yysym);
  }

#if YYDEBUG
  template <typename Base>
  void
   FuzzyParser ::yy_print_ (std::ostream& yyo,
                                     const basic_symbol<Base>& yysym) const
  {
    std::ostream& yyoutput = yyo;
    YYUSE (yyoutput);
    symbol_number_type yytype = yysym.type_get ();
    yyo << (yytype < yyntokens_ ? "token" : "nterm")
        << ' ' << yytname_[yytype] << " ("
        << yysym.location << ": ";
    YYUSE (yytype);
    yyo << ')';
  }
#endif

  inline
  void
   FuzzyParser ::yypush_ (const char* m, state_type s, symbol_type& sym)
  {
    stack_symbol_type t (s, sym);
    yypush_ (m, t);
  }

  inline
  void
   FuzzyParser ::yypush_ (const char* m, stack_symbol_type& s)
  {
    if (m)
      YY_SYMBOL_PRINT (m, s);
    yystack_.push (s);
  }

  inline
  void
   FuzzyParser ::yypop_ (unsigned int n)
  {
    yystack_.pop (n);
  }

#if YYDEBUG
  std::ostream&
   FuzzyParser ::debug_stream () const
  {
    return *yycdebug_;
  }

  void
   FuzzyParser ::set_debug_stream (std::ostream& o)
  {
    yycdebug_ = &o;
  }


   FuzzyParser ::debug_level_type
   FuzzyParser ::debug_level () const
  {
    return yydebug_;
  }

  void
   FuzzyParser ::set_debug_level (debug_level_type l)
  {
    yydebug_ = l;
  }
#endif // YYDEBUG

  inline  FuzzyParser ::state_type
   FuzzyParser ::yy_lr_goto_state_ (state_type yystate, int yysym)
  {
    int yyr = yypgoto_[yysym - yyntokens_] + yystate;
    if (0 <= yyr && yyr <= yylast_ && yycheck_[yyr] == yystate)
      return yytable_[yyr];
    else
      return yydefgoto_[yysym - yyntokens_];
  }

  inline bool
   FuzzyParser ::yy_pact_value_is_default_ (int yyvalue)
  {
    return yyvalue == yypact_ninf_;
  }

  inline bool
   FuzzyParser ::yy_table_value_is_error_ (int yyvalue)
  {
    return yyvalue == yytable_ninf_;
  }

  int
   FuzzyParser ::parse ()
  {
    /// Whether yyla contains a lookahead.
    bool yyempty = true;

    // State.
    int yyn;
    /// Length of the RHS of the rule being reduced.
    int yylen = 0;

    // Error handling.
    int yynerrs_ = 0;
    int yyerrstatus_ = 0;

    /// The lookahead symbol.
    symbol_type yyla;

    /// The locations where the error started and ended.
    stack_symbol_type yyerror_range[3];

    /// The return value of parse ().
    int yyresult;

    // FIXME: This shoud be completely indented.  It is not yet to
    // avoid gratuitous conflicts when merging into the master branch.
    try
      {
    YYCDEBUG << "Starting parse" << std::endl;


    /* Initialize the stack.  The initial state will be set in
       yynewstate, since the latter expects the semantical and the
       location values to have been already stored, initialize these
       stacks with a primary value.  */
    yystack_.clear ();
    yypush_ (YY_NULLPTR, 0, yyla);

    // A new symbol was pushed on the stack.
  yynewstate:
    YYCDEBUG << "Entering state " << yystack_[0].state << std::endl;

    // Accept?
    if (yystack_[0].state == yyfinal_)
      goto yyacceptlab;

    goto yybackup;

    // Backup.
  yybackup:

    // Try to take a decision without lookahead.
    yyn = yypact_[yystack_[0].state];
    if (yy_pact_value_is_default_ (yyn))
      goto yydefault;

    // Read a lookahead token.
    if (yyempty)
      {
        YYCDEBUG << "Reading a token: ";
        try
          {
            symbol_type yylookahead (yylex ());
            yyla.move (yylookahead);
          }
        catch (const syntax_error& yyexc)
          {
            error (yyexc);
            goto yyerrlab1;
          }
        yyempty = false;
      }
    YY_SYMBOL_PRINT ("Next token is", yyla);

    /* If the proper action on seeing token YYLA.TYPE is to reduce or
       to detect an error, take that action.  */
    yyn += yyla.type_get ();
    if (yyn < 0 || yylast_ < yyn || yycheck_[yyn] != yyla.type_get ())
      goto yydefault;

    // Reduce or error.
    yyn = yytable_[yyn];
    if (yyn <= 0)
      {
        if (yy_table_value_is_error_ (yyn))
          goto yyerrlab;
        yyn = -yyn;
        goto yyreduce;
      }

    // Discard the token being shifted.
    yyempty = true;

    // Count tokens shifted since error; after three, turn off error status.
    if (yyerrstatus_)
      --yyerrstatus_;

    // Shift the lookahead token.
    yypush_ ("Shifting", yyn, yyla);
    goto yynewstate;

  /*-----------------------------------------------------------.
  | yydefault -- do the default action for the current state.  |
  `-----------------------------------------------------------*/
  yydefault:
    yyn = yydefact_[yystack_[0].state];
    if (yyn == 0)
      goto yyerrlab;
    goto yyreduce;

  /*-----------------------------.
  | yyreduce -- Do a reduction.  |
  `-----------------------------*/
  yyreduce:
    yylen = yyr2_[yyn];
    {
      stack_symbol_type yylhs;
      yylhs.state = yy_lr_goto_state_(yystack_[yylen].state, yyr1_[yyn]);
      /* Variants are always initialized to an empty instance of the
         correct type. The default '$$ = $1' action is NOT applied
         when using variants.  */
        switch (yyr1_[yyn])
    {
      case 47: // wellFormedFormula
      case 48: // fuzzyComparison
      case 49: // fuzzyPredicateCall
      case 50: // fuzzyAssignment
        yylhs.value.build< NodePtr > ();
        break;

      case 51: // variable
        yylhs.value.build< Variable > ();
        break;

      case 25: // PARAMETER
        yylhs.value.build< int > ();
        break;

      case 3: // ID
      case 4: // VAR_ID
      case 24: // F_LABEL
      case 54: // var
      case 55: // templateVar
        yylhs.value.build< std::string > ();
        break;

      case 52: // variableList
      case 53: // variableListHelper
        yylhs.value.build< std::vector<Variable> > ();
        break;

      case 43: // shape
      case 44: // parametersList
        yylhs.value.build< std::vector<int> > ();
        break;

      case 41: // fuzzyId
      case 56: // templateVarList
      case 57: // templateVarListHelper
        yylhs.value.build< std::vector<std::string> > ();
        break;

      default:
        break;
    }


      // Compute the default @$.
      {
        slice<stack_symbol_type, stack_type> slice (yystack_, yylen);
        YYLLOC_DEFAULT (yylhs.location, slice, yylen);
      }

      // Perform the reduction.
      YY_REDUCE_PRINT (yyn);
      try
        {
          switch (yyn)
            {
  case 7:
#line 101 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    { builder.setNameSpace(yystack_[0].value.as< std::string > ()); }
#line 636 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 8:
#line 102 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				builder.setDefaultNameSpace();
			}
#line 644 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 12:
#line 112 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    { builder.enterPredicate(yystack_[0].value.as< std::vector<std::string> > ()); }
#line 650 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 13:
#line 113 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				builder.exitPredicate();
			}
#line 658 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 16:
#line 123 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				builder.buildPredicate(yystack_[3].value.as< std::string > (), yystack_[1].value.as< NodePtr > ());
			}
#line 666 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 19:
#line 133 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    { builder.buildDomain(yystack_[0].value.as< std::string > ()); }
#line 672 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 21:
#line 136 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    { builder.buildDomain(yystack_[0].value.as< std::vector<std::string> > ()); }
#line 678 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 23:
#line 140 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::vector<std::string> > ().push_back(yystack_[0].value.as< std::string > ());
			}
#line 686 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 24:
#line 144 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::vector<std::string> > () = yystack_[0].value.as< std::vector<std::string> > ();
				yylhs.value.as< std::vector<std::string> > ().push_back(yystack_[2].value.as< std::string > ());
			}
#line 695 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 25:
#line 151 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				builder.buildMF(yystack_[4].value.as< std::string > (), yystack_[2].value.as< std::string > (), yystack_[1].value.as< std::vector<int> > ());
			}
#line 703 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 26:
#line 155 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				builder.buildMF(yystack_[5].value.as< std::string > (), yystack_[3].value.as< std::string > (), yystack_[2].value.as< std::vector<int> > ());
			}
#line 711 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 27:
#line 161 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::vector<int> > () = yystack_[1].value.as< std::vector<int> > ();
			}
#line 719 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 28:
#line 167 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    { 
				yylhs.value.as< std::vector<int> > ().push_back(yystack_[0].value.as< int > ());
			}
#line 727 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 29:
#line 171 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    { 
				yylhs.value.as< std::vector<int> > () = yystack_[0].value.as< std::vector<int> > ();
				yylhs.value.as< std::vector<int> > ().push_back(yystack_[2].value.as< int > ());
			}
#line 736 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 32:
#line 183 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				builder.buildRule(yystack_[2].value.as< NodePtr > (), yystack_[1].value.as< NodePtr > ());
			}
#line 744 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 33:
#line 189 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< NodePtr > () = yystack_[0].value.as< NodePtr > ();
			}
#line 752 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 34:
#line 193 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< NodePtr > () = yystack_[0].value.as< NodePtr > ();
			}
#line 760 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 35:
#line 197 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< NodePtr > () = yystack_[1].value.as< NodePtr > ();
			}
#line 768 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 36:
#line 201 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< NodePtr > () = builder.buildNot(yystack_[0].value.as< NodePtr > ());
			}
#line 776 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 37:
#line 205 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< NodePtr > () = builder.buildOr(yystack_[2].value.as< NodePtr > (), yystack_[0].value.as< NodePtr > ());
			}
#line 784 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 38:
#line 209 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< NodePtr > () = builder.buildAnd(yystack_[2].value.as< NodePtr > (),yystack_[0].value.as< NodePtr > ());
			}
#line 792 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 39:
#line 215 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< NodePtr > () = builder.buildIs(yystack_[3].value.as< Variable > (), yystack_[1].value.as< std::string > ());
			}
#line 800 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 40:
#line 219 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< NodePtr > () = builder.buildTemplateIs(yystack_[3].value.as< std::string > (), yystack_[1].value.as< std::string > ());
			}
#line 808 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 41:
#line 225 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< NodePtr > () = builder.getPredicateInstance(yystack_[5].value.as< std::string > (), yystack_[3].value.as< std::string > (), yystack_[1].value.as< std::vector<Variable> > ());
			}
#line 816 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 42:
#line 229 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< NodePtr > () = builder.getPredicateInstance(yystack_[3].value.as< std::string > (), yystack_[1].value.as< std::vector<Variable> > ());
			}
#line 824 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 43:
#line 235 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< NodePtr > () = builder.buildAssignment(yystack_[3].value.as< Variable > (), yystack_[1].value.as< std::string > ());
			}
#line 832 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 44:
#line 241 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< Variable > ().nameSpace = yystack_[2].value.as< std::string > ();
				yylhs.value.as< Variable > ().domain = yystack_[0].value.as< std::string > ();
			}
#line 841 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 45:
#line 246 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< Variable > ().nameSpace = "";
				yylhs.value.as< Variable > ().domain = yystack_[0].value.as< std::string > ();
			}
#line 850 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 46:
#line 253 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::vector<Variable> > () = yystack_[0].value.as< std::vector<Variable> > ();
			}
#line 858 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 47:
#line 257 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
			
			}
#line 866 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 48:
#line 263 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::vector<Variable> > () = yystack_[0].value.as< std::vector<Variable> > ();
				yylhs.value.as< std::vector<Variable> > ().push_back(yystack_[2].value.as< Variable > ());
			}
#line 875 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 49:
#line 268 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::vector<Variable> > ().push_back(yystack_[0].value.as< Variable > ());
			}
#line 883 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 50:
#line 276 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::string > () = yystack_[0].value.as< std::string > ();
			}
#line 891 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 51:
#line 280 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::string > () = yystack_[0].value.as< std::string > ();
			}
#line 899 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 52:
#line 286 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::string > () = yystack_[0].value.as< std::string > ();
			}
#line 907 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 53:
#line 292 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::vector<std::string> > () = yystack_[0].value.as< std::vector<std::string> > ();
			}
#line 915 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 54:
#line 296 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
			
			}
#line 923 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 55:
#line 302 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::vector<std::string> > () = yystack_[0].value.as< std::vector<std::string> > ();
				yylhs.value.as< std::vector<std::string> > ().push_back(yystack_[2].value.as< std::string > ());
			}
#line 932 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;

  case 56:
#line 307 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::vector<std::string> > ().push_back(yystack_[0].value.as< std::string > ());
			}
#line 940 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
    break;


#line 944 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:847
            default:
              break;
            }
        }
      catch (const syntax_error& yyexc)
        {
          error (yyexc);
          YYERROR;
        }
      YY_SYMBOL_PRINT ("-> $$ =", yylhs);
      yypop_ (yylen);
      yylen = 0;
      YY_STACK_PRINT ();

      // Shift the result of the reduction.
      yypush_ (YY_NULLPTR, yylhs);
    }
    goto yynewstate;

  /*--------------------------------------.
  | yyerrlab -- here on detecting error.  |
  `--------------------------------------*/
  yyerrlab:
    // If not already recovering from an error, report this error.
    if (!yyerrstatus_)
      {
        ++yynerrs_;
        error (yyla.location, yysyntax_error_ (yystack_[0].state,
                                           yyempty ? yyempty_ : yyla.type_get ()));
      }


    yyerror_range[1].location = yyla.location;
    if (yyerrstatus_ == 3)
      {
        /* If just tried and failed to reuse lookahead token after an
           error, discard it.  */

        // Return failure if at end of input.
        if (yyla.type_get () == yyeof_)
          YYABORT;
        else if (!yyempty)
          {
            yy_destroy_ ("Error: discarding", yyla);
            yyempty = true;
          }
      }

    // Else will try to reuse lookahead token after shifting the error token.
    goto yyerrlab1;


  /*---------------------------------------------------.
  | yyerrorlab -- error raised explicitly by YYERROR.  |
  `---------------------------------------------------*/
  yyerrorlab:

    /* Pacify compilers like GCC when the user code never invokes
       YYERROR and the label yyerrorlab therefore never appears in user
       code.  */
    if (false)
      goto yyerrorlab;
    yyerror_range[1].location = yystack_[yylen - 1].location;
    /* Do not reclaim the symbols of the rule whose action triggered
       this YYERROR.  */
    yypop_ (yylen);
    yylen = 0;
    goto yyerrlab1;

  /*-------------------------------------------------------------.
  | yyerrlab1 -- common code for both syntax error and YYERROR.  |
  `-------------------------------------------------------------*/
  yyerrlab1:
    yyerrstatus_ = 3;   // Each real token shifted decrements this.
    {
      stack_symbol_type error_token;
      for (;;)
        {
          yyn = yypact_[yystack_[0].state];
          if (!yy_pact_value_is_default_ (yyn))
            {
              yyn += yyterror_;
              if (0 <= yyn && yyn <= yylast_ && yycheck_[yyn] == yyterror_)
                {
                  yyn = yytable_[yyn];
                  if (0 < yyn)
                    break;
                }
            }

          // Pop the current state because it cannot handle the error token.
          if (yystack_.size () == 1)
            YYABORT;

          yyerror_range[1].location = yystack_[0].location;
          yy_destroy_ ("Error: popping", yystack_[0]);
          yypop_ ();
          YY_STACK_PRINT ();
        }

      yyerror_range[2].location = yyla.location;
      YYLLOC_DEFAULT (error_token.location, yyerror_range, 2);

      // Shift the error token.
      error_token.state = yyn;
      yypush_ ("Shifting", error_token);
    }
    goto yynewstate;

    // Accept.
  yyacceptlab:
    yyresult = 0;
    goto yyreturn;

    // Abort.
  yyabortlab:
    yyresult = 1;
    goto yyreturn;

  yyreturn:
    if (!yyempty)
      yy_destroy_ ("Cleanup: discarding lookahead", yyla);

    /* Do not reclaim the symbols of the rule whose action triggered
       this YYABORT or YYACCEPT.  */
    yypop_ (yylen);
    while (1 < yystack_.size ())
      {
        yy_destroy_ ("Cleanup: popping", yystack_[0]);
        yypop_ ();
      }

    return yyresult;
  }
    catch (...)
      {
        YYCDEBUG << "Exception caught: cleaning lookahead and stack"
                 << std::endl;
        // Do not try to display the values of the reclaimed symbols,
        // as their printer might throw an exception.
        if (!yyempty)
          yy_destroy_ (YY_NULLPTR, yyla);

        while (1 < yystack_.size ())
          {
            yy_destroy_ (YY_NULLPTR, yystack_[0]);
            yypop_ ();
          }
        throw;
      }
  }

  void
   FuzzyParser ::error (const syntax_error& yyexc)
  {
    error (yyexc.location, yyexc.what());
  }

  // Generate an error message.
  std::string
   FuzzyParser ::yysyntax_error_ (state_type yystate, symbol_number_type yytoken) const
  {
    std::string yyres;
    // Number of reported tokens (one for the "unexpected", one per
    // "expected").
    size_t yycount = 0;
    // Its maximum.
    enum { YYERROR_VERBOSE_ARGS_MAXIMUM = 5 };
    // Arguments of yyformat.
    char const *yyarg[YYERROR_VERBOSE_ARGS_MAXIMUM];

    /* There are many possibilities here to consider:
       - If this state is a consistent state with a default action, then
         the only way this function was invoked is if the default action
         is an error action.  In that case, don't check for expected
         tokens because there are none.
       - The only way there can be no lookahead present (in yytoken) is
         if this state is a consistent state with a default action.
         Thus, detecting the absence of a lookahead is sufficient to
         determine that there is no unexpected or expected token to
         report.  In that case, just report a simple "syntax error".
       - Don't assume there isn't a lookahead just because this state is
         a consistent state with a default action.  There might have
         been a previous inconsistent state, consistent state with a
         non-default action, or user semantic action that manipulated
         yyla.  (However, yyla is currently not documented for users.)
       - Of course, the expected token list depends on states to have
         correct lookahead information, and it depends on the parser not
         to perform extra reductions after fetching a lookahead from the
         scanner and before detecting a syntax error.  Thus, state
         merging (from LALR or IELR) and default reductions corrupt the
         expected token list.  However, the list is correct for
         canonical LR with one exception: it will still contain any
         token that will not be accepted due to an error action in a
         later state.
    */
    if (yytoken != yyempty_)
      {
        yyarg[yycount++] = yytname_[yytoken];
        int yyn = yypact_[yystate];
        if (!yy_pact_value_is_default_ (yyn))
          {
            /* Start YYX at -YYN if negative to avoid negative indexes in
               YYCHECK.  In other words, skip the first -YYN actions for
               this state because they are default actions.  */
            int yyxbegin = yyn < 0 ? -yyn : 0;
            // Stay within bounds of both yycheck and yytname.
            int yychecklim = yylast_ - yyn + 1;
            int yyxend = yychecklim < yyntokens_ ? yychecklim : yyntokens_;
            for (int yyx = yyxbegin; yyx < yyxend; ++yyx)
              if (yycheck_[yyx + yyn] == yyx && yyx != yyterror_
                  && !yy_table_value_is_error_ (yytable_[yyx + yyn]))
                {
                  if (yycount == YYERROR_VERBOSE_ARGS_MAXIMUM)
                    {
                      yycount = 1;
                      break;
                    }
                  else
                    yyarg[yycount++] = yytname_[yyx];
                }
          }
      }

    char const* yyformat = YY_NULLPTR;
    switch (yycount)
      {
#define YYCASE_(N, S)                         \
        case N:                               \
          yyformat = S;                       \
        break
        YYCASE_(0, YY_("syntax error"));
        YYCASE_(1, YY_("syntax error, unexpected %s"));
        YYCASE_(2, YY_("syntax error, unexpected %s, expecting %s"));
        YYCASE_(3, YY_("syntax error, unexpected %s, expecting %s or %s"));
        YYCASE_(4, YY_("syntax error, unexpected %s, expecting %s or %s or %s"));
        YYCASE_(5, YY_("syntax error, unexpected %s, expecting %s or %s or %s or %s"));
#undef YYCASE_
      }

    // Argument number.
    size_t yyi = 0;
    for (char const* yyp = yyformat; *yyp; ++yyp)
      if (yyp[0] == '%' && yyp[1] == 's' && yyi < yycount)
        {
          yyres += yytnamerr_ (yyarg[yyi++]);
          ++yyp;
        }
      else
        yyres += *yyp;
    return yyres;
  }


  const signed char  FuzzyParser ::yypact_ninf_ = -93;

  const signed char  FuzzyParser ::yytable_ninf_ = -1;

  const signed char
   FuzzyParser ::yypact_[] =
  {
      24,    42,    30,     5,    39,    28,    24,    24,    24,   -93,
     -93,   -93,    27,   -93,    42,    41,   -93,   -93,   -93,     9,
     -93,    28,   -93,   -93,   -93,    60,    42,    16,   -93,     5,
      61,    -3,     9,     6,    25,   -93,   -93,   -93,    46,    50,
     -93,    51,    16,    16,   -93,    49,    56,    61,    53,    64,
     -93,     2,    37,    59,   -93,    63,     9,     9,    67,    68,
      48,   -93,   -93,   -93,   -93,     9,     5,    58,    56,   -93,
      57,    62,    70,   -93,    69,    55,   -93,    78,    79,   -93,
      80,    53,   -93,    75,    45,   -93,   -93,   -93,    42,    53,
     -93,    53,    69,   -93,    81,    82,    73,    65,    83,   -93,
      60,   -93,    84,   -93,   -93,    86,    72,    85,    60,    87,
     -93,    88,    65,   -93,   -93,   -93,   -93,   -93
  };

  const unsigned char
   FuzzyParser ::yydefact_[] =
  {
       6,     0,     0,    54,     0,    31,     6,     6,     6,    50,
      51,    21,    23,     7,     0,    56,    12,    53,     1,     0,
       2,    31,     3,     5,     4,     0,     0,    11,    52,     0,
       0,     0,     0,     0,     0,    33,    34,    30,     0,     0,
      24,     0,    11,    11,    55,     0,    18,    15,    47,     0,
      36,    50,     0,     0,    45,     0,     0,     0,     0,     0,
       0,    22,     8,    10,     9,     0,     0,     0,    18,    14,
      50,    49,     0,    46,     0,     0,    35,     0,     0,    37,
      38,     0,    32,     0,     0,    19,    13,    17,     0,     0,
      42,    47,    50,    44,     0,     0,     0,     0,     0,    16,
       0,    48,     0,    39,    40,     0,    28,     0,    25,     0,
      41,     0,     0,    27,    26,    20,    43,    29
  };

  const signed char
   FuzzyParser ::yypgoto_[] =
  {
     -93,   -93,    47,   -93,   -93,    18,   -22,   -93,    52,   -93,
      29,   -93,   -93,   -20,   -93,    74,   -92,   -93,   -16,    89,
     -93,   -30,   -93,   -93,   -93,   -32,    10,    14,    -1,   -29,
     -93,    76
  };

  const signed char
   FuzzyParser ::yydefgoto_[] =
  {
      -1,     4,     5,     6,    27,    41,     7,    30,    46,    47,
      67,    68,   100,     8,    25,    11,    39,    98,   107,    20,
      21,    34,    35,    36,    59,    71,    72,    73,    54,    15,
      16,    17
  };

  const unsigned char
   FuzzyParser ::yytable_[] =
  {
      12,    53,    50,    52,    55,    42,    48,    43,   109,    51,
      10,    48,    31,    28,    32,    33,   114,    32,    33,    49,
      42,    42,    43,    43,    75,    12,    79,    80,    14,    14,
       1,    56,    57,    13,     3,    84,    58,    85,     1,    18,
       2,    19,     3,    56,    57,     9,    10,    76,    26,    96,
      99,    56,    57,    22,    23,    24,    70,    10,    92,    10,
      63,    64,    29,    38,    45,    61,    60,    74,    62,    65,
      66,    77,    83,    82,    93,    78,    81,    86,    91,    88,
      90,    94,    95,    89,    97,   105,    56,    93,   108,   111,
     106,   103,   104,   112,   110,   113,   117,    87,   116,    69,
      40,   102,   115,   101,     0,    44,     0,     0,     0,     0,
      37
  };

  const signed char
   FuzzyParser ::yycheck_[] =
  {
       1,    33,    32,    33,    33,    27,     9,    27,   100,     3,
       4,     9,     3,    14,     8,     9,   108,     8,     9,    22,
      42,    43,    42,    43,    22,    26,    56,    57,    23,    23,
      14,     6,     7,     3,    18,    65,    11,    66,    14,     0,
      16,    13,    18,     6,     7,     3,     4,    10,    21,    81,
       5,     6,     7,     6,     7,     8,     3,     4,     3,     4,
      42,    43,    21,     3,     3,    15,    20,     3,    17,    20,
      14,    12,    24,     5,    75,    12,     9,    19,     9,    22,
      10,     3,     3,    21,     9,    12,     6,    88,     5,     3,
      25,    10,    10,    21,    10,    10,   112,    68,    10,    47,
      26,    91,    15,    89,    -1,    29,    -1,    -1,    -1,    -1,
      21
  };

  const unsigned char
   FuzzyParser ::yystos_[] =
  {
       0,    14,    16,    18,    27,    28,    29,    32,    39,     3,
       4,    41,    54,     3,    23,    55,    56,    57,     0,    13,
      45,    46,    28,    28,    28,    40,    21,    30,    54,    21,
      33,     3,     8,     9,    47,    48,    49,    45,     3,    42,
      41,    31,    32,    39,    57,     3,    34,    35,     9,    22,
      47,     3,    47,    51,    54,    55,     6,     7,    11,    50,
      20,    15,    17,    31,    31,    20,    14,    36,    37,    34,
       3,    51,    52,    53,     3,    22,    10,    12,    12,    47,
      47,     9,     5,    24,    47,    55,    19,    36,    22,    21,
      10,     9,     3,    54,     3,     3,    51,     9,    43,     5,
      38,    53,    52,    10,    10,    12,    25,    44,     5,    42,
      10,     3,    21,    10,    42,    15,    10,    44
  };

  const unsigned char
   FuzzyParser ::yyr1_[] =
  {
       0,    26,    27,    28,    28,    28,    28,    30,    29,    31,
      31,    31,    33,    32,    34,    34,    35,    36,    36,    38,
      37,    40,    39,    41,    41,    42,    42,    43,    44,    44,
      45,    45,    46,    47,    47,    47,    47,    47,    47,    48,
      48,    49,    49,    50,    51,    51,    52,    52,    53,    53,
      54,    54,    55,    56,    56,    57,    57
  };

  const unsigned char
   FuzzyParser ::yyr2_[] =
  {
       0,     2,     2,     2,     2,     2,     0,     0,     5,     2,
       2,     0,     0,     6,     2,     1,     4,     2,     0,     0,
       5,     0,     5,     1,     3,     5,     6,     3,     1,     3,
       2,     0,     4,     1,     1,     3,     2,     3,     3,     5,
       5,     6,     4,     6,     3,     1,     1,     0,     3,     1,
       1,     1,     2,     1,     0,     3,     1
  };



  // YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
  // First, the terminals, then, starting at \a yyntokens_, nonterminals.
  const char*
  const  FuzzyParser ::yytname_[] =
  {
  "END", "error", "$undefined", "ID", "VAR_ID", "END_RULE", "OP_OR",
  "OP_AND", "OP_NOT", "OPEN_B", "CLOSE_B", "THEN", "IS", "IF", "FUZZIFY",
  "END_FUZZIFY", "FUZZIFY_CLASS", "END_FUZZIFY_CLASS", "FUZZIFY_PREDICATE",
  "END_FUZZIFY_PREDICATE", "LIKE", "COMMA", "PERIOD", "QUESTION",
  "F_LABEL", "PARAMETER", "$accept", "fuzzyFile", "fuzzyDefinitions",
  "fuzzyClass", "$@1", "fuzzyClassDefinitions", "fuzzyPredicate", "$@2",
  "fuzzyPredicateList", "fuzzyPredicateDef", "fuzzytemplateSetList",
  "fuzzyTemplateSet", "$@3", "fuzzySet", "$@4", "fuzzyId", "fuzzyTerm",
  "shape", "parametersList", "ruleSet", "rule", "wellFormedFormula",
  "fuzzyComparison", "fuzzyPredicateCall", "fuzzyAssignment", "variable",
  "variableList", "variableListHelper", "var", "templateVar",
  "templateVarList", "templateVarListHelper", YY_NULLPTR
  };

#if YYDEBUG
  const unsigned short int
   FuzzyParser ::yyrline_[] =
  {
       0,    92,    92,    95,    96,    97,    98,   101,   101,   107,
     108,   109,   112,   112,   118,   119,   122,   129,   130,   133,
     133,   136,   136,   139,   143,   150,   154,   160,   166,   170,
     177,   178,   182,   188,   192,   196,   200,   204,   208,   214,
     218,   224,   228,   234,   240,   245,   252,   257,   262,   267,
     275,   279,   285,   291,   296,   301,   306
  };

  // Print the state stack on the debug stream.
  void
   FuzzyParser ::yystack_print_ ()
  {
    *yycdebug_ << "Stack now";
    for (stack_type::const_iterator
           i = yystack_.begin (),
           i_end = yystack_.end ();
         i != i_end; ++i)
      *yycdebug_ << ' ' << i->state;
    *yycdebug_ << std::endl;
  }

  // Report on the debug stream that the rule \a yyrule is going to be reduced.
  void
   FuzzyParser ::yy_reduce_print_ (int yyrule)
  {
    unsigned int yylno = yyrline_[yyrule];
    int yynrhs = yyr2_[yyrule];
    // Print the symbols being reduced, and their result.
    *yycdebug_ << "Reducing stack by rule " << yyrule - 1
               << " (line " << yylno << "):" << std::endl;
    // The symbols being reduced.
    for (int yyi = 0; yyi < yynrhs; yyi++)
      YY_SYMBOL_PRINT ("   $" << yyi + 1 << " =",
                       yystack_[(yynrhs) - (yyi + 1)]);
  }
#endif // YYDEBUG


#line 5 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:1155
} // fz
#line 1393 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.tab.cpp" // lalr1.cc:1155
#line 312 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_fuzzy/FuzzyParser.y" // lalr1.cc:1156


void fz::FuzzyParser::error(const fz::FuzzyParser::location_type& l, const std::string& msg)
{
	std::stringstream ss;
	ss << "Error: " << msg << ", between " << l.begin << " and " << l.end << std::endl;
	throw std::runtime_error(ss.str());
}

