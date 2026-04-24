# First-order STL

This specification extends the syntax of RTAMT with *all* and *exists* quantifiers. We also use antlr to automatically create a parser for the extended syntax.

## Syntax

### Vehicle variables

A vehicle variable refers to an arbitrary vehicle towards which the rule is evaluated. It is denoted by the letter `a` followed by a positive integer index. By convention, the ego vehicle is always referred to as `a0`.

### Predicates

The predicate name consists of letters and underscores. It is followed by the comma-separated list of vehicle variables. If the predicate is an input-predicate as defined by IA-STL, `_i` is appended.

Examples:

```
# Unary predicate
in_standstill(a0)

# Binary predicate
in_front_of(a0, a1)

# Input predicate
in_standstill(a0)_i
```

### Quantification

Vehicle variables, except the ego `a0`, always have to be bound by a quantifier. This can either be the *all*-quantifier `A` or the *existential*-quantifier `E`. The quantifier is follower by the bound vehicle variable. A colon before a parenthesized expression indicates the scope of the bound expression.

Examples:

```
A a1: (in_front_of(a0, a1))
E a1: (in_front_of(a0, a1))

# Invalid, a2 is not bound.
A a1: (in_front_of(a0, a2))

# Invalid, a1 is bound by the scope of the parantheses
E a1: (in_standstill(a0)) and in_front_of(a0, a1)
```

## Generating the parser

Modification of the grammar requires re-generating the parser.
The parser can be generated using the command

```bash
antlr4 -visitor -no-listener -Dlanguage=Python3 -lib ../../../external/rtamt/rtamt/antlr/grammar/tl -lib ../../../external/rtamt/rtamt/antlr/parser/stl FaStlLexer.g4 FaStlParser.g4
```

Installation instructions for antlr can be found [here](https://www.antlr.org/download.html).