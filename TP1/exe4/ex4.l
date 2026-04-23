%{
#include <stdio.h>
#include <string.h>
%}

DIGIT   [0-9]
LETTER  [a-zA-Z_]
ALNUM   [a-zA-Z0-9_]
ID      {LETTER}{ALNUM}*

%%

[Cc][Rr][Ee][Aa][Tt][Ee]                       { printf("%-20s CREATE\n",      yytext); }
[Tt][Aa][Bb][Ll][Ee]                            { printf("%-20s TABLE\n",       yytext); }
[Pp][Rr][Ii][Mm][Aa][Rr][Yy][ \t]+[Kk][Ee][Yy] { printf("%-20s PRIMARY_KEY\n", yytext); }
[Ii][Nn][Tt]                                    { printf("%-20s DATATYPE\n",    yytext); }
[Vv][Aa][Rr][Cc][Hh][Aa][Rr]                   { printf("%-20s DATATYPE\n",    yytext); }
[Dd][Aa][Tt][Ee]                                { printf("%-20s DATATYPE\n",    yytext); }
{DIGIT}+                                        { printf("%-20s NUMBER\n",      yytext); }
","                                             { printf("%-20s COMMA\n",       yytext); }
"("                                             { printf("%-20s (\n",           yytext); }
")"                                             { printf("%-20s )\n",           yytext); }
{ID}                                            { printf("%-20s IDENTIFIER\n",  yytext); }
[ \t\n]                                         { }
.                                               { }

%%

int yywrap() { return 1; }

int main(int argc, char *argv[])
{
    ++argv; --argc;
    if (argc > 0)
        yyin = fopen(argv[0], "r");
    else
        yyin = stdin;
    yylex();
    return 0;
}
