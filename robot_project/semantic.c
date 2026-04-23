#include "semantic.h"
#include <stdio.h>

/* Starting state used as the reference for simulating the command sequence. */
static EtatRobot g_sem_start = {0, 0, NORD, 10, 10, 0};

/* Store the current state before semantic validation. */
void semantic_set_state(const EtatRobot *state) {
    if (state) g_sem_start = *state;
}

/* Check that the sequence stays coherent: positive distances, valid grid, no out-of-bounds moves. */
int semantic_check(const Commande *cmds, int n, char *err, size_t err_sz) {
    int trace_id = log_trace_get();
    if (!cmds || n < 0) {
        if (err && err_sz) snprintf(err, err_sz, "Erreur semantique: sequence invalide");
        runtime_log("[ERR][SEM][%d] invalid sequence", trace_id);
        return 0;
    }

    EtatRobot s = g_sem_start;
    if (s.grille_w <= 0 || s.grille_h <= 0) {
        if (err && err_sz) snprintf(err, err_sz, "Erreur semantique: dimensions de grille invalides");
        runtime_log("[ERR][SEM][%d] invalid grid size | w=%d h=%d", trace_id, s.grille_w, s.grille_h);
        return 0;
    }

    for (int i = 0; i < n; i++) {
        const Commande c = cmds[i];

        switch (c.type) {
        case CMD_AVANCER:
        case CMD_RECULER: {
            /* Movement commands must always use a strictly positive distance. */
            if (c.valeur <= 0) {
                if (err && err_sz) {
                    snprintf(err, err_sz,
                             "Erreur semantique: distance <= 0 a la commande %d", i + 1);
                }
                runtime_log("[ERR][SEM][%d] distance <= 0 | cmd_index=%d value=%d", trace_id, i + 1, c.valeur);
                return 0;
            }

            /* Convert the current direction into a movement vector. */
            int dx = 0, dy = 0;
            if (s.direction == NORD) dy = 1;
            else if (s.direction == EST) dx = 1;
            else if (s.direction == SUD) dy = -1;
            else if (s.direction == OUEST) dx = -1;

            int nx = s.x + (c.type == CMD_AVANCER ? dx : -dx) * c.valeur;
            int ny = s.y + (c.type == CMD_AVANCER ? dy : -dy) * c.valeur;

            /* Prevent the simulated robot from leaving the grid. */
            if (nx < 0 || nx >= s.grille_w || ny < 0 || ny >= s.grille_h) {
                if (err && err_sz) {
                    snprintf(err, err_sz,
                             "Erreur semantique: sortie de grille a la commande %d (%d,%d)",
                             i + 1, nx, ny);
                }
                runtime_log("[ERR][SEM][%d] out of grid | cmd_index=%d from=(%d,%d) to=(%d,%d) grid=%dx%d",
                            trace_id, i + 1, s.x, s.y, nx, ny, s.grille_w, s.grille_h);
                return 0;
            }

            s.x = nx;
            s.y = ny;
            break;
        }

        /* Rotation commands only change the orientation. */
        case CMD_TOURNER_DROITE:
            s.direction = (s.direction + 1) % 4;
            break;
        case CMD_TOURNER_GAUCHE:
            s.direction = (s.direction + 3) % 4;
            break;
        case CMD_DEMITOUR:
            s.direction = (s.direction + 2) % 4;
            break;
        /* Display and reset do not alter the logical robot state. */
        case CMD_AFFICHER:
        case CMD_RESET:
            break;
        default:
            if (err && err_sz) {
                snprintf(err, err_sz,
                         "Erreur semantique: type de commande invalide (%d)", (int)c.type);
            }
            runtime_log("[ERR][SEM][%d] invalid command type | type=%d", trace_id, (int)c.type);
            return 0;
        }
    }

    if (err && err_sz) err[0] = '\0';
    runtime_log("[OK][SEM][%d] sequence valid | actions=%d", trace_id, n);
    return 1;
}
