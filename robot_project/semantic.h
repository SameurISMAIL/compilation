#ifndef SEMANTIC_H
#define SEMANTIC_H

#include <stddef.h>
#include "robot_core.h"

/* Set the starting state used by semantic validation. */
void semantic_set_state(const EtatRobot *state);

/*
 * Validate a sequence of already-parsed commands semantically.
 * Returns 1 if valid, 0 otherwise, and fills err.
 */
int semantic_check(const Commande *cmds, int n, char *err, size_t err_sz);

#endif
