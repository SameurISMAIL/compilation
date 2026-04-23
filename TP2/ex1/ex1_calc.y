%{
#include <stdio.h>
int yylex(void);
int yyerror(char *s);
%}

%union {
    int entier;
}

%token <entier> NB
%token FIN SOM PROD
%type <entier> listesom listeprod

%%

liste : FIN
              { printf("correct\n"); }
      | SOM listesom '.' liste
              { printf("Somme = %d\n", $2); }
      | PROD listeprod '.' liste
              { printf("Produit = %d\n", $2); }
;

listesom  : NB
              { $$ = $1; }
          | listesom ',' NB
              { $$ = $1 + $3; }
;

listeprod : NB
              { $$ = $1; }
          | listeprod ',' NB
              { $$ = $1 * $3; }
;

%%

#include "lex.yy.c"

int yyerror(char *s) {
    printf("%s\n", s);
    return 0;
}

int main() {
    yyparse();
    return 0;
}