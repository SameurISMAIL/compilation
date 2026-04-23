%{
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

int ligne = 1;
%}

DIGIT    [0-9]
LETTRE   [a-zA-Z_]
ID       {LETTRE}[a-zA-Z0-9_]*
ENTIER   {DIGIT}+
REEL     [+-]?{DIGIT}+\.{DIGIT}+([eE][+-]?{DIGIT}+)?
REEL2    [+-]?\.{DIGIT}+([eE][+-]?{DIGIT}+)?
CHAINE   \"[^\"\n]*\"

%%

{CHAINE}  { printf("CHAINE    : %s\n", yytext); }

\"[^\"\n]*\n {
    printf("** Erreur : ligne %d ** chaine incomplete, fin de chaine attendue\n", ligne);
    ligne++;
}

"if"|"else"|"while"          { printf("MOTCLE    : %s\n", yytext); }

"<="|">="|"=="|"!="          { printf("OP-REL    : %s\n", yytext); }
"<"|">"                      { printf("OP-REL    : %s\n", yytext); }

"+"|"-"|"*"|"/"              { printf("OP-ARTHM  : %s\n", yytext); }

"="                          { printf("AFFECT    : %s\n", yytext); }

{REEL}                       { printf("REEL      : %f\n", atof(yytext)); }
{REEL2}                      { printf("REEL      : %f\n", atof(yytext)); }
{ENTIER}                     { printf("ENTIER    : %s\n", yytext); }

{DIGIT}+[eE][a-zA-Z_][a-zA-Z0-9_]* {
    printf("** Erreur : ligne %d ** exposant attendu apres e/E : [%s]\n", ligne, yytext);
}

{DIGIT}+\.?{DIGIT}*[eE] {
    printf("** Erreur : ligne %d ** exposant attendu apres e/E : [%s]\n", ligne, yytext);
}

{DIGIT}+{LETTRE}[a-zA-Z0-9_]* {
    printf("** Erreur : ligne %d ** identificateur mal forme : [%s]\n", ligne, yytext);
}

"#"[^\n]*                    { printf("COMMENT   : %s\n", yytext); }

{ID}                         { printf("IDENT     : %s\n", yytext); }

[+-]{LETTRE}[a-zA-Z0-9_]* {
    printf("** Erreur : ligne %d ** nombre attendu apres signe : [%s]\n", ligne, yytext);
}

[ \t]+                       { /* ignorer espaces */ }
\n                           { ligne++; }

.  { printf("** Erreur : ligne %d ** caractere invalide : [%s]\n", ligne, yytext); }

%%

int yywrap() { return 1; }

int main(int argc, char *argv[]) {
    if (argc > 1)
        yyin = fopen(argv[1], "r");
    else
        yyin = stdin;
    yylex();
    return 0;
}
