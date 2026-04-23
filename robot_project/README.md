# Car Interpreter — 3D (Bison + Flex + Raylib)

Interactive robot-command interpreter in C with:
- lexical + syntax parsing (Flex/Bison),
- semantic validation pass,
- real-time 3D visualization (Raylib),
- rich UI panels (history, lexemes, console, batch, AST viewer).

---

## Features

- Single-line command input (REPL style).
- Multiline batch execution panel.
- 3D robot/grid animation with deferred logical commit.
- Lexemes panel (`TYPE`, `VALEUR`, `LIGNE`) fed by the lexer.
- Structured console trace across stages: `LEX`, `SYN`, `SEM`, `RUN`.
- Semantic analysis pass (distance checks, out-of-grid checks, invalid command checks).
- AST modal viewer with hierarchy tree + symbol table.
- UI controls: zoom in/out, pause camera rotation, AST toggle button.

## Tech Stack

- C / gcc
- Flex + Bison
- Raylib

## Requirements

- `gcc`, `make`
- `flex`, `bison`
- Raylib dev package

Debian/Ubuntu example:

```bash
sudo apt install build-essential flex bison libraylib-dev pkg-config
```

## Build

```bash
make clean && make
```

Build output executable: `./robot`.

## Run

```bash
./robot
```

## Controls

- `Enter`: execute single-line command (when input focused)
- `Ctrl+B`: focus batch editor
- `Ctrl+I`: focus single-line input
- `Ctrl+R`: execute batch block
- `Ctrl+L`: enqueue reset
- `Ctrl +` / `Ctrl -`: UI zoom in/out
- `ESC`: quit
- Mouse buttons in HUD:
	- `+` / `-` zoom
	- pause/resume rotation
	- `AST` open/close AST interface

## Command Language

Supported commands (case-insensitive):

- `avancer N`
- `reculer N`
- `tourner droite`
- `tourner gauche`
- `tourner droite N`
- `tourner gauche N`
- `tourner demi-tour`
- `afficher`
- `reset`

Example:

```text
avancer 2
tourner droite
avancer 1
```

## Pipeline

1. **LEX** (`robot.l`): tokenization + lexeme capture for UI.
2. **SYN** (`robot.y`): grammar parse into `parsed_cmds[]`.
3. **SEM** (`semantic.c`): semantic validation against current state.
4. **RUN** (`robot3d.c`): enqueue + animate commands in 3D.

Console messages follow this format:

```text
[LEVEL][STAGE][ID] message | key=value
```

Example:

```text
[INF][LEX][17] token | type=AVANCER value='avancer' line=1
[OK][SYN][17] parse success | actions=1
[OK][SEM][17] sequence valid | actions=1
[INF][RUN][17] start | type=0 val=2 target=(0,2) world=(0.00,4.00)
```

## Project Structure

- `robot3d.c`: main UI, input, console, AST viewer, 3D rendering, runtime execution.
- `robot.l`: Flex lexer + lexeme tracing.
- `robot.y`: Bison grammar + parse driver.
- `semantic.c` / `semantic.h`: semantic analyzer module.
- `robot_core.h`: shared types and APIs.
- `Makefile`: build rules.

## Notes

- Semantic checks run **before** commands are accepted.
- Batch semantic checks are state-aware line-by-line.
- AST view is generated from successfully parsed commands and displayed in a dedicated modal.

## License

No license file is currently included.
