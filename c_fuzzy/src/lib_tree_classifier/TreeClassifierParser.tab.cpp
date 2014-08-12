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

#line 37 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:399

# ifndef YY_NULLPTR
#  if defined __cplusplus && 201103L <= __cplusplus
#   define YY_NULLPTR nullptr
#  else
#   define YY_NULLPTR 0
#  endif
# endif

#include "TreeClassifierParser.tab.h"

// User implementation prologue.

#line 51 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:407
// Unqualified %code blocks.
#line 32 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:408

	#include<fstream>

	#include "TreeClassifierBuilder.h"
	#include "TreeClassifierParser.tab.h"
	#include "TreeClassifierScanner.h"

	
	#undef yylex
	#define yylex scanner.lex
	

#line 66 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:408


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

#line 5 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:474
namespace tc {
#line 152 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:474

  /* Return YYSTR after stripping away unnecessary quotes and
     backslashes, so that it's suitable for yyerror.  The heuristic is
     that double-quoting is unnecessary unless the string contains an
     apostrophe, a comma, or backslash (other than backslash-backslash).
     YYSTR is taken from yytname.  */
  std::string
   TreeClassifierParser ::yytnamerr_ (const char *yystr)
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
   TreeClassifierParser :: TreeClassifierParser  (TreeClassifierBuilder  &builder_yyarg, tc::TreeClassifierScanner  &scanner_yyarg)
    :
#if YYDEBUG
      yydebug_ (false),
      yycdebug_ (&std::cerr),
#endif
      builder (builder_yyarg),
      scanner (scanner_yyarg)
  {}

   TreeClassifierParser ::~ TreeClassifierParser  ()
  {}


  /*---------------.
  | Symbol types.  |
  `---------------*/



  // by_state.
  inline
   TreeClassifierParser ::by_state::by_state ()
    : state (empty)
  {}

  inline
   TreeClassifierParser ::by_state::by_state (const by_state& other)
    : state (other.state)
  {}

  inline
  void
   TreeClassifierParser ::by_state::move (by_state& that)
  {
    state = that.state;
    that.state = empty;
  }

  inline
   TreeClassifierParser ::by_state::by_state (state_type s)
    : state (s)
  {}

  inline
   TreeClassifierParser ::symbol_number_type
   TreeClassifierParser ::by_state::type_get () const
  {
    return state == empty ? 0 : yystos_[state];
  }

  inline
   TreeClassifierParser ::stack_symbol_type::stack_symbol_type ()
  {}


  inline
   TreeClassifierParser ::stack_symbol_type::stack_symbol_type (state_type s, symbol_type& that)
    : super_type (s, that.location)
  {
      switch (that.type_get ())
    {
      case 29: // constants
      case 30: // constantList
        value.move< ConstantList* > (that.value);
        break;

      case 28: // fuzzyClassElements
        value.move< ElementsList > (that.value);
        break;

      case 34: // fuzzyFeature
        value.move< FuzzyFeatureData > (that.value);
        break;

      case 33: // fuzzyFeatures
        value.move< FuzzyFeatureList* > (that.value);
        break;

      case 31: // variables
      case 32: // variableList
        value.move< VariableList* > (that.value);
        break;

      case 27: // hiddenFlag
        value.move< bool > (that.value);
        break;

      case 3: // ID
      case 4: // VAR_ID
      case 26: // fuzzySuperclass
      case 39: // fuzzyConstraint
      case 40: // fuzzyDegree
      case 41: // var
        value.move< std::string > (that.value);
        break;

      case 35: // fuzzySimpleFeature
      case 36: // fuzzySimpleRelation
      case 37: // fuzzyComplexRelation
      case 38: // fuzzyInverseRelation
        value.move< std::vector<std::string>  > (that.value);
        break;

      default:
        break;
    }

    // that is emptied.
    that.type = empty;
  }

  inline
   TreeClassifierParser ::stack_symbol_type&
   TreeClassifierParser ::stack_symbol_type::operator= (const stack_symbol_type& that)
  {
    state = that.state;
      switch (that.type_get ())
    {
      case 29: // constants
      case 30: // constantList
        value.copy< ConstantList* > (that.value);
        break;

      case 28: // fuzzyClassElements
        value.copy< ElementsList > (that.value);
        break;

      case 34: // fuzzyFeature
        value.copy< FuzzyFeatureData > (that.value);
        break;

      case 33: // fuzzyFeatures
        value.copy< FuzzyFeatureList* > (that.value);
        break;

      case 31: // variables
      case 32: // variableList
        value.copy< VariableList* > (that.value);
        break;

      case 27: // hiddenFlag
        value.copy< bool > (that.value);
        break;

      case 3: // ID
      case 4: // VAR_ID
      case 26: // fuzzySuperclass
      case 39: // fuzzyConstraint
      case 40: // fuzzyDegree
      case 41: // var
        value.copy< std::string > (that.value);
        break;

      case 35: // fuzzySimpleFeature
      case 36: // fuzzySimpleRelation
      case 37: // fuzzyComplexRelation
      case 38: // fuzzyInverseRelation
        value.copy< std::vector<std::string>  > (that.value);
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
   TreeClassifierParser ::yy_destroy_ (const char* yymsg, basic_symbol<Base>& yysym) const
  {
    if (yymsg)
      YY_SYMBOL_PRINT (yymsg, yysym);
  }

#if YYDEBUG
  template <typename Base>
  void
   TreeClassifierParser ::yy_print_ (std::ostream& yyo,
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
   TreeClassifierParser ::yypush_ (const char* m, state_type s, symbol_type& sym)
  {
    stack_symbol_type t (s, sym);
    yypush_ (m, t);
  }

  inline
  void
   TreeClassifierParser ::yypush_ (const char* m, stack_symbol_type& s)
  {
    if (m)
      YY_SYMBOL_PRINT (m, s);
    yystack_.push (s);
  }

  inline
  void
   TreeClassifierParser ::yypop_ (unsigned int n)
  {
    yystack_.pop (n);
  }

#if YYDEBUG
  std::ostream&
   TreeClassifierParser ::debug_stream () const
  {
    return *yycdebug_;
  }

  void
   TreeClassifierParser ::set_debug_stream (std::ostream& o)
  {
    yycdebug_ = &o;
  }


   TreeClassifierParser ::debug_level_type
   TreeClassifierParser ::debug_level () const
  {
    return yydebug_;
  }

  void
   TreeClassifierParser ::set_debug_level (debug_level_type l)
  {
    yydebug_ = l;
  }
#endif // YYDEBUG

  inline  TreeClassifierParser ::state_type
   TreeClassifierParser ::yy_lr_goto_state_ (state_type yystate, int yysym)
  {
    int yyr = yypgoto_[yysym - yyntokens_] + yystate;
    if (0 <= yyr && yyr <= yylast_ && yycheck_[yyr] == yystate)
      return yytable_[yyr];
    else
      return yydefgoto_[yysym - yyntokens_];
  }

  inline bool
   TreeClassifierParser ::yy_pact_value_is_default_ (int yyvalue)
  {
    return yyvalue == yypact_ninf_;
  }

  inline bool
   TreeClassifierParser ::yy_table_value_is_error_ (int yyvalue)
  {
    return yyvalue == yytable_ninf_;
  }

  int
   TreeClassifierParser ::parse ()
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
      case 29: // constants
      case 30: // constantList
        yylhs.value.build< ConstantList* > ();
        break;

      case 28: // fuzzyClassElements
        yylhs.value.build< ElementsList > ();
        break;

      case 34: // fuzzyFeature
        yylhs.value.build< FuzzyFeatureData > ();
        break;

      case 33: // fuzzyFeatures
        yylhs.value.build< FuzzyFeatureList* > ();
        break;

      case 31: // variables
      case 32: // variableList
        yylhs.value.build< VariableList* > ();
        break;

      case 27: // hiddenFlag
        yylhs.value.build< bool > ();
        break;

      case 3: // ID
      case 4: // VAR_ID
      case 26: // fuzzySuperclass
      case 39: // fuzzyConstraint
      case 40: // fuzzyDegree
      case 41: // var
        yylhs.value.build< std::string > ();
        break;

      case 35: // fuzzySimpleFeature
      case 36: // fuzzySimpleRelation
      case 37: // fuzzyComplexRelation
      case 38: // fuzzyInverseRelation
        yylhs.value.build< std::vector<std::string>  > ();
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
  case 4:
#line 87 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				builder.buildClass(yystack_[5].value.as< std::string > (), yystack_[4].value.as< std::string > (), yystack_[2].value.as< ElementsList > ().first, yystack_[2].value.as< ElementsList > ().second, yystack_[1].value.as< FuzzyFeatureList* > (), yystack_[3].value.as< bool > ());
			}
#line 649 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 5:
#line 93 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::string > () = yystack_[0].value.as< std::string > ();
			}
#line 657 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 6:
#line 97 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::string > () = "";
			}
#line 665 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 7:
#line 103 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< bool > () = true;
			}
#line 673 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 8:
#line 107 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< bool > () = false;
			}
#line 681 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 9:
#line 113 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< ElementsList > ().first = yystack_[0].value.as< VariableList* > ();
				yylhs.value.as< ElementsList > ().second = yystack_[1].value.as< ConstantList* > ();
			}
#line 690 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 10:
#line 118 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< ElementsList > ().first = yystack_[1].value.as< VariableList* > ();
				yylhs.value.as< ElementsList > ().second = yystack_[0].value.as< ConstantList* > ();
			}
#line 699 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 11:
#line 123 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< ElementsList > ().first = NULL;
				yylhs.value.as< ElementsList > ().second = yystack_[0].value.as< ConstantList* > ();
			}
#line 708 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 12:
#line 128 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< ElementsList > ().first = yystack_[0].value.as< VariableList* > ();
				yylhs.value.as< ElementsList > ().second = NULL;
			}
#line 717 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 13:
#line 133 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< ElementsList > ().first = NULL;
				yylhs.value.as< ElementsList > ().second = NULL;
			}
#line 726 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 14:
#line 140 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< ConstantList* > () = yystack_[1].value.as< ConstantList* > ();
			}
#line 734 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 15:
#line 146 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< ConstantList* > () = builder.buildCostantList(yystack_[0].value.as< ConstantList* > (), yystack_[4].value.as< std::string > (), yystack_[2].value.as< std::string > ());

			}
#line 743 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 16:
#line 151 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< ConstantList* > () = NULL;
			}
#line 751 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 17:
#line 157 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< VariableList* > () = yystack_[1].value.as< VariableList* > ();
			}
#line 759 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 18:
#line 163 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< VariableList* > () = builder.buildVariableList(yystack_[0].value.as< VariableList* > (), yystack_[2].value.as< std::string > ());
			}
#line 767 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 19:
#line 167 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< VariableList* > () = NULL;
			}
#line 775 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 20:
#line 174 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< FuzzyFeatureList* > () = builder.buildFeaturesList(yystack_[0].value.as< FuzzyFeatureList* > (), yystack_[2].value.as< FuzzyFeatureData > ().first, yystack_[2].value.as< FuzzyFeatureData > ().second);
			}
#line 783 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 21:
#line 178 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< FuzzyFeatureList* > () = NULL;
			}
#line 791 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 22:
#line 184 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< FuzzyFeatureData > ().first = yystack_[0].value.as< std::vector<std::string>  > ();
				yylhs.value.as< FuzzyFeatureData > ().second = SIM_F;
			}
#line 800 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 23:
#line 189 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< FuzzyFeatureData > ().first = yystack_[0].value.as< std::vector<std::string>  > ();
				yylhs.value.as< FuzzyFeatureData > ().second = SIM_R;
			}
#line 809 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 24:
#line 194 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< FuzzyFeatureData > ().first = yystack_[0].value.as< std::vector<std::string>  > ();
				yylhs.value.as< FuzzyFeatureData > ().second = COM_R;
			}
#line 818 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 25:
#line 199 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< FuzzyFeatureData > ().first = yystack_[0].value.as< std::vector<std::string>  > ();
				yylhs.value.as< FuzzyFeatureData > ().second = INV_R;
			}
#line 827 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 26:
#line 206 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::vector<std::string>  > ().push_back(yystack_[2].value.as< std::string > ());
				yylhs.value.as< std::vector<std::string>  > ().push_back(yystack_[0].value.as< std::string > ());
			}
#line 836 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 27:
#line 213 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::vector<std::string>  > ().push_back(yystack_[5].value.as< std::string > ());
				yylhs.value.as< std::vector<std::string>  > ().push_back(yystack_[3].value.as< std::string > ());
				yylhs.value.as< std::vector<std::string>  > ().push_back(yystack_[1].value.as< std::string > ());
				yylhs.value.as< std::vector<std::string>  > ().push_back(yystack_[0].value.as< std::string > ());
			}
#line 847 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 28:
#line 222 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::vector<std::string>  > ().push_back(yystack_[9].value.as< std::string > ());
				yylhs.value.as< std::vector<std::string>  > ().push_back(yystack_[7].value.as< std::string > ());
				yylhs.value.as< std::vector<std::string>  > ().push_back(yystack_[3].value.as< std::string > ());
				yylhs.value.as< std::vector<std::string>  > ().push_back(yystack_[1].value.as< std::string > ());
				yylhs.value.as< std::vector<std::string>  > ().push_back(yystack_[6].value.as< std::string > ());
			}
#line 859 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 29:
#line 232 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::vector<std::string>  > ().push_back(yystack_[8].value.as< std::string > ());
				yylhs.value.as< std::vector<std::string>  > ().push_back(yystack_[5].value.as< std::string > ());
				yylhs.value.as< std::vector<std::string>  > ().push_back(yystack_[3].value.as< std::string > ());
				yylhs.value.as< std::vector<std::string>  > ().push_back(yystack_[1].value.as< std::string > ());
				yylhs.value.as< std::vector<std::string>  > ().push_back(yystack_[7].value.as< std::string > ());
			}
#line 871 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 30:
#line 243 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::string > () = yystack_[0].value.as< std::string > ();
			}
#line 879 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 31:
#line 247 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::string > () = "";
			}
#line 887 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 32:
#line 253 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::string > () = yystack_[0].value.as< std::string > ();
			}
#line 895 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 33:
#line 257 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::string > () = "";
			}
#line 903 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 34:
#line 264 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::string > () = yystack_[0].value.as< std::string > ();
			}
#line 911 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;

  case 35:
#line 268 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:847
    {
				yylhs.value.as< std::string > () = yystack_[0].value.as< std::string > ();
			}
#line 919 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
    break;


#line 923 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:847
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
   TreeClassifierParser ::error (const syntax_error& yyexc)
  {
    error (yyexc.location, yyexc.what());
  }

  // Generate an error message.
  std::string
   TreeClassifierParser ::yysyntax_error_ (state_type yystate, symbol_number_type yytoken) const
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


  const signed char  TreeClassifierParser ::yypact_ninf_ = -14;

  const signed char  TreeClassifierParser ::yytable_ninf_ = -31;

  const signed char
   TreeClassifierParser ::yypact_[] =
  {
      -6,     6,    12,    -6,    -2,   -14,   -14,    11,    -1,   -14,
     -14,    -8,     2,     2,     4,     7,     8,   -14,   -14,    10,
       0,     3,    -3,    13,     9,    14,   -14,   -14,   -14,   -14,
      17,   -14,   -14,   -14,     2,   -14,    22,     2,   -14,     4,
      24,    21,   -14,    15,     5,   -14,    27,    26,     2,    28,
       2,    29,    18,   -14,   -14,    31,    20,     2,    38,   -14,
       2,    23,   -14,    30,     2,     2,    25,    32,   -14,   -14
  };

  const unsigned char
   TreeClassifierParser ::yydefact_[] =
  {
       0,     0,     0,     3,     6,     1,     2,     0,     8,     5,
       7,    13,    19,    16,    21,    11,    12,    34,    35,     0,
       0,     0,     0,    34,     0,     0,    22,    23,    24,    25,
      31,     9,    10,    17,    19,    14,     0,     0,     4,    21,
       0,     0,    18,     0,    31,    20,    26,     0,    16,     0,
       0,     0,     0,    15,    30,    33,     0,     0,     0,    27,
       0,     0,    32,     0,     0,     0,     0,     0,    29,    28
  };

  const signed char
   TreeClassifierParser ::yypgoto_[] =
  {
     -14,    40,   -14,   -14,   -14,   -14,    34,    16,    33,    35,
      19,   -14,   -14,   -14,   -14,   -14,     1,   -14,   -13
  };

  const signed char
   TreeClassifierParser ::yydefgoto_[] =
  {
      -1,     2,     3,     8,    11,    14,    15,    21,    16,    19,
      24,    25,    26,    27,    28,    29,    41,    59,    20
  };

  const signed char
   TreeClassifierParser ::yytable_[] =
  {
      22,    30,    12,     1,    13,    17,    18,    23,    18,     4,
      49,    50,     5,     7,     9,    10,    35,    12,    34,    36,
      13,    33,    40,    38,    44,    43,    30,    46,    47,    52,
      37,    54,    39,    48,   -30,    22,    56,    55,    57,    58,
      60,    62,    64,     6,    61,    51,    68,    63,    31,    65,
      32,    66,    67,    69,     0,     0,     0,     0,    45,     0,
       0,     0,     0,     0,    53,     0,     0,     0,     0,    42
  };

  const signed char
   TreeClassifierParser ::yycheck_[] =
  {
      13,    14,    10,     9,    12,     3,     4,     3,     4,     3,
       5,     6,     0,    15,     3,    16,    13,    10,    18,    22,
      12,    11,     5,    14,    37,     3,    39,     3,     7,     3,
      17,     3,    18,    18,     7,    48,     7,    50,    20,     8,
      20,     3,    19,     3,    57,    44,    21,    60,    15,    19,
      16,    64,    65,    21,    -1,    -1,    -1,    -1,    39,    -1,
      -1,    -1,    -1,    -1,    48,    -1,    -1,    -1,    -1,    34
  };

  const unsigned char
   TreeClassifierParser ::yystos_[] =
  {
       0,     9,    24,    25,     3,     0,    24,    15,    26,     3,
      16,    27,    10,    12,    28,    29,    31,     3,     4,    32,
      41,    30,    41,     3,    33,    34,    35,    36,    37,    38,
      41,    31,    29,    11,    18,    13,    22,    17,    14,    18,
       5,    39,    32,     3,    41,    33,     3,     7,    18,     5,
       6,    39,     3,    30,     3,    41,     7,    20,     8,    40,
      20,    41,     3,    41,    19,    19,    41,    41,    21,    21
  };

  const unsigned char
   TreeClassifierParser ::yyr1_[] =
  {
       0,    23,    24,    24,    25,    26,    26,    27,    27,    28,
      28,    28,    28,    28,    29,    30,    30,    31,    32,    32,
      33,    33,    34,    34,    34,    34,    35,    36,    37,    38,
      39,    39,    40,    40,    41,    41
  };

  const unsigned char
   TreeClassifierParser ::yyr2_[] =
  {
       0,     2,     2,     1,     7,     2,     0,     1,     0,     2,
       2,     1,     1,     0,     3,     5,     0,     3,     3,     0,
       3,     0,     1,     1,     1,     1,     3,     6,    10,     9,
       2,     0,     2,     0,     1,     1
  };



  // YYTNAME[SYMBOL-NUM] -- String name of the symbol SYMBOL-NUM.
  // First, the terminals, then, starting at \a yyntokens_, nonterminals.
  const char*
  const  TreeClassifierParser ::yytname_[] =
  {
  "END", "error", "$undefined", "ID", "VAR_ID", "IS", "MATCH", "ON",
  "DEGREE", "CLASS", "VARIABLES", "END_VARIABLES", "CONSTANTS",
  "END_CONSTANTS", "END_CLASS", "EXTENDS", "HIDDEN", "PERIOD", "SEMICOLON",
  "COMMA", "LPAR", "RPAR", "EQUAL", "$accept", "fuzzyClassifiers",
  "fuzzyClass", "fuzzySuperclass", "hiddenFlag", "fuzzyClassElements",
  "constants", "constantList", "variables", "variableList",
  "fuzzyFeatures", "fuzzyFeature", "fuzzySimpleFeature",
  "fuzzySimpleRelation", "fuzzyComplexRelation", "fuzzyInverseRelation",
  "fuzzyConstraint", "fuzzyDegree", "var", YY_NULLPTR
  };

#if YYDEBUG
  const unsigned short int
   TreeClassifierParser ::yyrline_[] =
  {
       0,    82,    82,    83,    86,    92,    97,   102,   107,   112,
     117,   122,   127,   133,   139,   145,   151,   156,   162,   167,
     173,   178,   183,   188,   193,   198,   205,   212,   221,   231,
     242,   247,   252,   257,   263,   267
  };

  // Print the state stack on the debug stream.
  void
   TreeClassifierParser ::yystack_print_ ()
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
   TreeClassifierParser ::yy_reduce_print_ (int yyrule)
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


#line 5 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:1155
} // tc
#line 1334 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.tab.cpp" // lalr1.cc:1155
#line 273 "/home/dave/CognitiveSlam/src/c_fuzzy/src/lib_tree_classifier/TreeClassifierParser.y" // lalr1.cc:1156


void tc::TreeClassifierParser::error(const tc::TreeClassifierParser::location_type& l, const std::string& msg)
{
	std::stringstream ss;
	ss << "Error: " << msg << ", between " << l.begin << " and " << l.end << std::endl;
	throw std::runtime_error(ss.str());
}

