%{
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "robot.tab.h"
#include "robot_core.h"
/* UI hook: push lexemes for inspection. */
/* UI bridge: each recognized lexeme is shown in the dedicated panel. */
extern void ui_add_lexeme(const char *type, const char *val, int line);

/* Flex lit dans un tampon fourni par l'interface au lieu de stdin. */
static const char *input_buf = NULL;
static int         input_pos = 0;
/* Current line number used to annotate lexemes shown in the UI. */
static int         lex_line = 1;

#undef  YY_INPUT
#define YY_INPUT(buf, result, max_size) \
    do { \
        if (input_buf && input_buf[input_pos]) { \
            buf[0] = input_buf[input_pos++]; \
            result = 1; \
        } else { \
            result = YY_NULL; \
        } \
    } while(0)

void lex_set_input(const char *s) {
    input_buf = s;
    input_pos = 0;
    YY_FLUSH_BUFFER;
    yyrestart(NULL);
}

/* Build a lexical error message and stop processing the input. */
static int lexical_error(){
    snprintf(parse_err, sizeof(parse_err), "Erreur lexicale: '%s'", yytext);
    runtime_log("[ERR][LEX][%d] lexical error | token='%s' line=%d", log_trace_get(), yytext, lex_line);
    return 0;
}
%}
static void lex_log_token(const char *type){
    runtime_log("[INF][LEX][%d] token | type=%s value='%s' line=%d", log_trace_get(), type, yytext, lex_line);
}
/* Command-language keywords. */
%}

%%
"avancer"       { ui_add_lexeme("AVANCER", yytext, lex_line); lex_log_token("AVANCER"); return AVANCER; }
"reculer"       { ui_add_lexeme("RECULER", yytext, lex_line); lex_log_token("RECULER"); return RECULER; }
"tourner"       { ui_add_lexeme("TOURNER", yytext, lex_line); lex_log_token("TOURNER"); return TOURNER; }
"droite"        { ui_add_lexeme("DROITE", yytext, lex_line); lex_log_token("DROITE"); return DROITE; }
"gauche"        { ui_add_lexeme("GAUCHE", yytext, lex_line); lex_log_token("GAUCHE"); return GAUCHE; }
"demi-tour"     { ui_add_lexeme("DEMITOUR", yytext, lex_line); lex_log_token("DEMITOUR"); return DEMITOUR; }
"afficher"      { ui_add_lexeme("AFFICHER", yytext, lex_line); lex_log_token("AFFICHER"); return AFFICHER; }
"reset"         { ui_add_lexeme("RESET", yytext, lex_line); lex_log_token("RESET"); return RESET_CMD; }
/* Integers: stored in yylval for the Bison parser. */
[0-9]+           { ui_add_lexeme("NUMBER", yytext, lex_line); lex_log_token("NUMBER"); yylval.entier = atoi(yytext); return NB; }
/* Whitespace and newlines: ignored, but line numbers are counted. */
[ \t\r\n]+     { /* Whitespace: count newlines. */
                    for(size_t i=0;i<strlen(yytext);i++) if(yytext[i]=='\n') lex_line++; }
/* Any unexpected identifier is treated as a lexical error. */
[a-zA-Z_][a-zA-Z0-9_]* { ui_add_lexeme("IDENT", yytext, lex_line); lex_log_token("IDENT"); return lexical_error(); }
/* Any other character is rejected. */
.                { ui_add_lexeme("UNKNOWN", yytext, lex_line); lex_log_token("UNKNOWN"); return lexical_error(); }
%%

int yywrap() { return 1; }
