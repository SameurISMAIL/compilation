%{
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "robot_core.h"
#include "semantic.h"

int  yylex(void);
int  yyerror(char *s);
void lex_set_input(const char *s);

/* Current parsing result: consumed later by robot3d.c for the AST and execution. */
Commande parsed_cmds[64];
int      parsed_n   = 0;
int      parse_ok   = 0;
char     parse_err[128] = "";

EtatRobot  etat_initial;
Commande   commandes_liste[MAX_COMMANDES];
int        nb_commandes = 0;

/* Add one command to the sequence produced by the parser. */
static void add(TypeCommande t, int v) {
    if (parsed_n < 64) {
        parsed_cmds[parsed_n].type  = t;
        parsed_cmds[parsed_n].valeur = v;
        parsed_n++;
    }
}
%}

%union { int entier; }

%token <entier> NB
%token AVANCER RECULER TOURNER AFFICHER RESET_CMD
%token DROITE GAUCHE DEMITOUR

%%

/* A valid input corresponds to a complete command. */
entree
    : commande          { if(parse_err[0] == '\0') parse_ok = 1; else parse_ok = 0; }
    | error             { parse_ok = 0; }
    ;

commande
    : AVANCER NB        { add(CMD_AVANCER,       $2); }
    | RECULER NB        { add(CMD_RECULER,        $2); }
    | TOURNER DROITE    { add(CMD_TOURNER_DROITE,  0); }
    | TOURNER GAUCHE    { add(CMD_TOURNER_GAUCHE,  0); }
    | TOURNER DROITE NB { add(CMD_TOURNER_DROITE,  0);
                          add(CMD_AVANCER,         $3); }
    | TOURNER GAUCHE NB { add(CMD_TOURNER_GAUCHE,  0);
                          add(CMD_AVANCER,         $3); }
    | TOURNER DEMITOUR  { add(CMD_DEMITOUR,         0); }
    | AFFICHER          { add(CMD_AFFICHER,         0); }
    | RESET_CMD         { add(CMD_RESET,            0); }
    ;

%%

#define YY_FATAL_ERROR(msg) runtime_log("Lexer fatal: %s", msg)
#include "lex.yy.c"
#undef YY_FATAL_ERROR

/* Entry point called by the UI: initialize parsing and then run the parser. */
int parse_command(const char *input) {
    int trace_id = log_trace_get();
    parsed_n = 0;
    parse_ok  = 0;
    parse_err[0] = '\0';
    lex_set_input(input);
    runtime_log("[INF][SYN][%d] parse start", trace_id);
    yyparse();
    if (!parse_ok) {
        runtime_log("[ERR][SYN][%d] parse failed | %s", trace_id, parse_err[0] ? parse_err : "syntax error");
    }
    if (parse_ok) {
        if (!semantic_check(parsed_cmds, parsed_n, parse_err, sizeof(parse_err))) {
            parse_ok = 0;
        }
    }
    if (parse_ok) runtime_log("[OK][SYN][%d] parse success | actions=%d", trace_id, parsed_n);
    return parse_ok;
}

int yyerror(char *s) {
    snprintf(parse_err, sizeof(parse_err), "Erreur : %s", s);
    parse_ok = 0;
    /* Do not forward here to avoid double-reporting; the caller will log. */
    return 0;
}