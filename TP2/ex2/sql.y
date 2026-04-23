%{
#include <stdio.h>
int yylex(void);
int yyerror(char *s);
%}

%union {
    int   num;
    char* str;
}

%token CREATE TABLE_KW PRIMARY_KEY
%token <str> IDENTIFIER DATATYPE
%token <num> NUMBER

%%

requete : CREATE TABLE_KW IDENTIFIER '(' liste_colonnes ')'
          {
              printf("Requete CREATE TABLE correcte !\n");
          }
        ;

liste_colonnes : colonne
               | liste_colonnes ',' colonne
               ;

colonne : IDENTIFIER DATATYPE
          {
              printf("  Colonne: %s  Type: %s\n", $1, $2);
          }
        | IDENTIFIER DATATYPE '(' NUMBER ')'
          {
              printf("  Colonne: %s  Type: %s(%d)\n", $1, $2, $4);
          }
        | IDENTIFIER DATATYPE PRIMARY_KEY
          {
              printf("  Colonne: %s  Type: %s  [PRIMARY KEY]\n", $1, $2);
          }
        ;

%%

int yyerror(char *s) {
    printf("Erreur syntaxique: %s\n", s);
    return 0;
}

int main() {
    yyparse();
    return 0;
}