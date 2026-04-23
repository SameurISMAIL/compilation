#ifndef ROBOT_CORE_H
#define ROBOT_CORE_H

/* Maximum number of commands stored by the interpreter. */
#define MAX_COMMANDES 512

/* Cardinal directions used by the robot state. */
#define NORD  0
#define EST   1
#define SUD   2
#define OUEST 3

/* Command types recognized by the parser and executor. */
typedef enum {
    CMD_AVANCER,
    CMD_RECULER,
    CMD_TOURNER_DROITE,
    CMD_TOURNER_GAUCHE,
    CMD_DEMITOUR,
    CMD_AFFICHER,
    CMD_RESET
} TypeCommande;

/* Representation of a parsed command: type plus an optional value. */
typedef struct {
    TypeCommande type;
    int          valeur;
} Commande;

/* Logical robot state used by semantic validation and animation. */
typedef struct {
    int x, y;
    int direction;
    int grille_w, grille_h;
    int erreur;
} EtatRobot;

/* Global history queue of commands. */
extern Commande  commandes_liste[MAX_COMMANDES];
extern int       nb_commandes;
extern EtatRobot etat_initial;

/* Current parsing result, read back by the graphical interface. */
extern Commande parsed_cmds[64];
extern int      parsed_n;
extern int      parse_ok;
extern char     parse_err[128];
int parse_command(const char *input);

/* Execution log provided by the UI. */
void runtime_log(const char *fmt, ...);

/* Shared trace identifier across lexer, parser, semantic checks, and runtime. */
int  log_trace_next(void);
void log_trace_set(int id);
int  log_trace_get(void);

#endif