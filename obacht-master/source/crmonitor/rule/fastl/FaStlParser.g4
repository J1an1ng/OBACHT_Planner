parser grammar FaStlParser ;
import StlParser;

options {
	tokenVocab = FaStlLexer ;
}

compile_unit:
	spec EOF;

vehicle
    : VEHICLE IntegerLiteral;

predicate
    : Identifier LPAREN vehicle (COMMA vehicle)* RPAREN IO_TYPE_INPUT?;

spec
	:
    real_expression                                       #SpecNested
	| LPAREN spec RPAREN                                  #SpecNested
	| NotOperator spec                                    #SpecNested

    | EXIST vehicle COLON LPAREN spec RPAREN              #SpecQuantExist
	| FORALL vehicle COLON LPAREN spec RPAREN             #SpecQuantForall

    | spec AndOperator spec                               #SpecNested
    | spec OrOperator spec                                #SpecNested
    | spec ImpliesOperator spec                           #SpecNested
    | spec IffOperator spec                               #SpecNested
    | spec XorOperator spec                               #SpecNested

	| AlwaysOperator ( interval )? spec                   #SpecNested
    | EventuallyOperator ( interval )? spec               #SpecNested
    | spec UntilOperator ( interval )? spec               #SpecNested
    | spec UnlessOperator ( interval )? spec              #SpecNested
    | HistoricallyOperator ( interval )? spec             #SpecNested
    | OnceOperator ( interval )? spec                     #SpecNested
    | spec SinceOperator ( interval )? spec               #SpecNested
    | RiseOperator LPAREN spec RPAREN                     #SpecNested
    | FallOperator LPAREN spec RPAREN                     #SpecNested
    | PreviousOperator spec                               #SpecNested
    | NextOperator spec                                   #SpecNested
	;

real_expression:
     literal                                                   #ExprLiteral
    | predicate                                                  #ExprPred
    | real_expression comparisonOp real_expression                          #ExprComp
    | real_expression PLUS real_expression                      #ExprAddition
	| real_expression MINUS real_expression                     #ExprSubtraction
	| real_expression TIMES real_expression                     #ExprMultiplication
	| real_expression DIVIDE real_expression                    #ExprDivision

	| ABS LPAREN real_expression RPAREN                         #ExprAbs
	| SQRT LPAREN real_expression RPAREN                        #ExprSqrt
	| EXP LPAREN real_expression RPAREN                         #ExprExp
	| POW LPAREN real_expression COMMA real_expression RPAREN   #ExprPow
	;