#include "raylib.h"
#include "raymath.h"
#include "robot_core.h"
#include "semantic.h"
#include "rlgl.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>

/* Compatibility helpers */
#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

/* ═══ CONFIG ═══════════════════════════════════ */
#define CELL       2.0f
#define SCR_W      1280
#define SCR_H      720
#define MAX_PATH   2048
#define HIST_MAX   20
#define INPUT_MAX  128
/* Multiline batch area */
#define BATCH_MAX  4096

/* ═══ COLORS ════════════════════════════════════ */
#define C_BG      ((Color){8,   8,  18, 255})
#define C_GA      ((Color){22,  22,  42, 255})
#define C_GB      ((Color){32,  32,  62, 255})
#define C_GL      ((Color){60,  60, 110, 255})
#define C_BOD     ((Color){0,  190, 255, 255})
#define C_BOD2    ((Color){0,  140, 200, 255})
#define C_CAB     ((Color){140,220, 255, 150})
#define C_WHL     ((Color){35,  35,  65, 255})
#define C_WHL2    ((Color){80,  80, 130, 255})
#define C_LDY     ((Color){255,220,   0, 255})
#define C_LDR     ((Color){255, 50,  50, 255})
#define C_LDG     ((Color){0,  255, 100, 255})
#define C_PTH     ((Color){0,  200, 255,  80})
#define C_ACC     ((Color){0,  190, 255, 255})
#define C_HUD     ((Color){0,    0,   0, 210})
#define C_INP     ((Color){12,  12,  28, 240})
#define C_OK      ((Color){0,  220,  80, 255})
#define C_ERR     ((Color){255, 60,  60, 255})
#define C_WARN    ((Color){255,200,   0, 255})
#define C_ORG     ((Color){0,  255, 100, 255})

/* ═══ STATE ═════════════════════════════════════ */
static float rob_x, rob_z, rob_angle;
static float tgt_x, tgt_z, tgt_angle;
static int   animating = 0;
static float wspin     = 0.f;
static float bob_t     = 0.f;

/* Execution queue */
#define EXEC_MAX 256
static Commande exec_q[EXEC_MAX];
static int      exec_head=0, exec_tail=0;

static EtatRobot cur;
/* State for the upcoming animation (applied, but not committed yet) */
static EtatRobot next_state;

/* Path */
typedef struct { float x,z; } P2;
static P2  path[MAX_PATH];
static int path_n = 0;

/* Command history */
typedef struct {
    char  text[INPUT_MAX];
    int   ok;         /* 1=success, 0=error */
} HistEntry;
static HistEntry hist[HIST_MAX];
static int       hist_n = 0;

/* UI log */
#define UI_LOG_MAX 32
static char ui_log_buf[UI_LOG_MAX][160];
static int  ui_log_n = 0;
static int  g_trace_id = 0;
static int  g_trace_seq = 0;

/* AST viewer */
#define AST_MAX_NODES   128
#define AST_MAX_SYMBOLS 64
typedef struct { char label[64]; int parent; float x,y; } AstNode;
typedef struct { int index; char symbol[48]; char type[24]; } AstSymbol;
static AstNode   ast_nodes[AST_MAX_NODES];
static int       ast_node_n = 0;
static AstSymbol ast_symbols[AST_MAX_SYMBOLS];
static int       ast_sym_n = 0;
static int       ast_visible = 0;

/* Lexeme inspection buffer (for UI display) */
#define LEX_MAX 256
typedef struct { char type[32]; char val[160]; int line; } LexEntry;
static LexEntry lexemes[LEX_MAX];
static int      lex_n = 0;

/* Called by the lexer or other code to push an observed lexeme for UI inspection. */
void ui_add_lexeme(const char *type, const char *val, int line){
    if(!type) return;
    if(lex_n < LEX_MAX){
        strncpy(lexemes[lex_n].type, type, sizeof lexemes[0].type - 1);
        strncpy(lexemes[lex_n].val,  val ?: "", sizeof lexemes[0].val - 1);
        lexemes[lex_n].line = line;
        lexemes[lex_n].type[sizeof lexemes[0].type - 1] = '\0';
        lexemes[lex_n].val[sizeof lexemes[0].val - 1] = '\0';
        lex_n++;
    } else {
        /* Rotate the buffer contents. */
        for(int i=1;i<LEX_MAX;i++) lexemes[i-1]=lexemes[i];
        strncpy(lexemes[LEX_MAX-1].type, type, sizeof lexemes[0].type - 1);
        strncpy(lexemes[LEX_MAX-1].val,  val ?: "", sizeof lexemes[0].val - 1);
        lexemes[LEX_MAX-1].line = line;
        lexemes[LEX_MAX-1].type[sizeof lexemes[0].type - 1] = '\0';
        lexemes[LEX_MAX-1].val[sizeof lexemes[0].val - 1] = '\0';
    }
}

static void ui_log(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    if(ui_log_n < UI_LOG_MAX) {
        vsnprintf(ui_log_buf[ui_log_n], sizeof ui_log_buf[0], fmt, ap);
        ui_log_n++;
    } else {
        /* Rotate the buffer contents. */
        for(int i=1;i<UI_LOG_MAX;i++) strcpy(ui_log_buf[i-1], ui_log_buf[i]);
        vsnprintf(ui_log_buf[UI_LOG_MAX-1], sizeof ui_log_buf[0], fmt, ap);
    }
    /* Also print to stdout to help debugging from the terminal. */
    {
        char tmp[256]; va_list ap2; va_start(ap2, fmt);
        vsnprintf(tmp, sizeof tmp, fmt, ap2);
        va_end(ap2);
        printf("[UI] %s\n", tmp);
        fflush(stdout);
    }
    va_end(ap);
}

/* Exposed runtime log for the lexer/parser (visible externally). */
void runtime_log(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt);
    char tmp[256];
    vsnprintf(tmp, sizeof tmp, fmt, ap);
    va_end(ap);
    ui_log("%s", tmp);
}

int log_trace_next(void){
    g_trace_seq++;
    if(g_trace_seq <= 0) g_trace_seq = 1;
    return g_trace_seq;
}

void log_trace_set(int id){
    g_trace_id = id;
}

int log_trace_get(void){
    return g_trace_id;
}

/* Input field */
static char  input_buf[INPUT_MAX] = "";
static int   input_len = 0;
static float cursor_blink = 0.f;
static int   input_focused = 1;

/* Batch (multiline) buffer */
static char  batch_buf[BATCH_MAX] = "";
static int   batch_len = 0;
static int   batch_focused = 0;
static int   batch_scroll = 0;
/* UI font (loaded at runtime if available) */
static Font ui_font;
/* UI scale for zooming text */
static float ui_scale = 1.6f;
/* Pause flag for grid/camera rotation */
static int grid_rotate_paused = 0;
/* Debounce timers (seconds) for keys to avoid double handling */
static float last_enter_time = -1.0f;
static float last_backspace_time = -1.0f;

static void draw_ui_text(const char *s, int x, int y, int size, Color col){
    Vector2 p = { (float)x, (float)y };
    float sz = (float)size * ui_scale;
    DrawTextEx(ui_font, s, p, sz, 1.0f * ui_scale, col);
}

static const char* cmd_type_name(TypeCommande t){
    switch(t){
        case CMD_AVANCER: return "AVANCER";
        case CMD_RECULER: return "RECULER";
        case CMD_TOURNER_DROITE: return "TOURNER_DROITE";
        case CMD_TOURNER_GAUCHE: return "TOURNER_GAUCHE";
        case CMD_DEMITOUR: return "DEMITOUR";
        case CMD_AFFICHER: return "AFFICHER";
        case CMD_RESET: return "RESET";
        default: return "UNKNOWN";
    }
}

static const char* ast_cmd_label(TypeCommande t){
    switch(t){
        case CMD_AVANCER: return "avancer";
        case CMD_RECULER: return "reculer";
        case CMD_TOURNER_DROITE: return "tourner_droite";
        case CMD_TOURNER_GAUCHE: return "tourner_gauche";
        case CMD_DEMITOUR: return "demi_tour";
        case CMD_AFFICHER: return "afficher";
        case CMD_RESET: return "reset";
        default: return "unknown";
    }
}

static int ast_add_node(const char *label, int parent){
    if(ast_node_n >= AST_MAX_NODES) return -1;
    int idx = ast_node_n++;
    strncpy(ast_nodes[idx].label, label?label:"", sizeof(ast_nodes[idx].label)-1);
    ast_nodes[idx].label[sizeof(ast_nodes[idx].label)-1] = '\0';
    ast_nodes[idx].parent = parent;
    ast_nodes[idx].x = ast_nodes[idx].y = 0.f;
    return idx;
}

static void ast_add_symbol_unique(const char *symbol, const char *type){
    if(!symbol || !symbol[0] || ast_sym_n >= AST_MAX_SYMBOLS) return;
    for(int i=0;i<ast_sym_n;i++) if(strcmp(ast_symbols[i].symbol, symbol)==0) return;
    ast_symbols[ast_sym_n].index = ast_sym_n + 1;
    strncpy(ast_symbols[ast_sym_n].symbol, symbol, sizeof(ast_symbols[0].symbol)-1);
    ast_symbols[ast_sym_n].symbol[sizeof(ast_symbols[0].symbol)-1] = '\0';
    strncpy(ast_symbols[ast_sym_n].type, type?type:"", sizeof(ast_symbols[0].type)-1);
    ast_symbols[ast_sym_n].type[sizeof(ast_symbols[0].type)-1] = '\0';
    ast_sym_n++;
}

static int ast_node_depth(int idx){
    int d=0;
    while(idx>=0 && ast_nodes[idx].parent>=0){ d++; idx = ast_nodes[idx].parent; }
    return d;
}

static void ast_layout_node(int idx, int *leaf, float x0, float dx, float y0, float dy){
    int first = *leaf;
    int child_count = 0;
    float sumx = 0.f;
    for(int i=0;i<ast_node_n;i++){
        if(ast_nodes[i].parent == idx){
            ast_layout_node(i, leaf, x0, dx, y0, dy);
            sumx += ast_nodes[i].x;
            child_count++;
        }
    }
    int depth = ast_node_depth(idx);
    ast_nodes[idx].y = y0 + depth * dy;
    if(child_count == 0){
        ast_nodes[idx].x = x0 + first * dx;
        (*leaf)++;
    } else {
        ast_nodes[idx].x = sumx / (float)child_count;
    }
}

static int ast_is_number(const char *s){
    if(!s || !s[0]) return 0;
    for(int i=0;s[i];i++) if(!isdigit((unsigned char)s[i])) return 0;
    return 1;
}

static int ast_is_keyword(const char *s){
    static const char *kw[] = {"avancer","reculer","tourner","droite","gauche","demi-tour","afficher","reset",0};
    for(int i=0;kw[i];i++) if(strcmp(s, kw[i])==0) return 1;
    return 0;
}

/* Add one line of commands to the same AST so the full batch is preserved. */
static void ast_accumulate_from_parsed(const char *source) {
    /* If the "commands" node does not exist yet, create the tree skeleton. */
    int cmds_node = -1;
    for (int i = 0; i < ast_node_n; i++) {
        if (strcmp(ast_nodes[i].label, "commands") == 0) {
            cmds_node = i;
            break;
        }
    }

    if (cmds_node == -1) {
        /* First call: initialize the tree. */
        ast_node_n = 0;
        ast_sym_n  = 0;
        int root = ast_add_node("program",  -1);
        int src  = ast_add_node("source",   root);
        cmds_node = ast_add_node("commands", root);
        char src_lbl[80];
        snprintf(src_lbl, sizeof(src_lbl), "%s", source && source[0] ? source : "(batch)");
        ast_add_node(src_lbl, src);
    }

    /* Each command becomes a child node; the value is added for advance/backward commands. */
    for (int i = 0; i < parsed_n; i++) {
        char cbuf[64];
        snprintf(cbuf, sizeof(cbuf), "%s", ast_cmd_label(parsed_cmds[i].type));
        int cnode = ast_add_node(cbuf, cmds_node);
        if (parsed_cmds[i].type == CMD_AVANCER || parsed_cmds[i].type == CMD_RECULER) {
            char vbuf[32];
            snprintf(vbuf, sizeof(vbuf), "%d", parsed_cmds[i].valeur);
            ast_add_node(vbuf, cnode);
        }
    }

    /* Also populate the symbol table for the AST panel. */
    ast_add_symbol_unique("program", "root");
    if (source && source[0]) {
        char tmp[INPUT_MAX];
        strncpy(tmp, source, sizeof(tmp)-1);
        tmp[sizeof(tmp)-1] = '\0';
        char *sp = 0;
        for (char *tok = strtok_r(tmp, " \t\r\n", &sp); tok; tok = strtok_r(NULL, " \t\r\n", &sp)) {
            char low[64]; int j = 0;
            for (; tok[j] && j < 63; j++) low[j] = (char)tolower((unsigned char)tok[j]);
            low[j] = '\0';
            if (ast_is_number(low))  ast_add_symbol_unique(tok, "number");
            else if (ast_is_keyword(low)) ast_add_symbol_unique(tok, "keyword");
            else ast_add_symbol_unique(tok, "identifier");
        }
    }
}

static void ast_build_from_parsed(const char *source){
    ast_node_n = 0;
    ast_sym_n = 0;

    int root = ast_add_node("program", -1);
    int src  = ast_add_node("source", root);
    int cmds = ast_add_node("commands", root);

    char src_lbl[80];
    snprintf(src_lbl, sizeof(src_lbl), "%s", source && source[0] ? source : "(batch line)");
    ast_add_node(src_lbl, src);

    for(int i=0;i<parsed_n;i++){
        char cbuf[64];
        snprintf(cbuf, sizeof(cbuf), "%s", ast_cmd_label(parsed_cmds[i].type));
        int cnode = ast_add_node(cbuf, cmds);
        if(parsed_cmds[i].type == CMD_AVANCER || parsed_cmds[i].type == CMD_RECULER){
            char vbuf[32]; snprintf(vbuf, sizeof(vbuf), "%d", parsed_cmds[i].valeur);
            ast_add_node(vbuf, cnode);
        }
    }

    ast_add_symbol_unique("program", "root");
    if(source && source[0]){
        char tmp[INPUT_MAX];
        strncpy(tmp, source, sizeof(tmp)-1);
        tmp[sizeof(tmp)-1] = '\0';
        char *sp=0;
        for(char *tok=strtok_r(tmp, " \t\r\n", &sp); tok; tok=strtok_r(NULL, " \t\r\n", &sp)){
            char low[64];
            int j=0;
            for(; tok[j] && j<63; j++) low[j] = (char)tolower((unsigned char)tok[j]);
            low[j]='\0';
            if(ast_is_number(low)) ast_add_symbol_unique(tok, "number");
            else if(ast_is_keyword(low)) ast_add_symbol_unique(tok, "keyword");
            else ast_add_symbol_unique(tok, "identifier");
        }
    }
}

static void draw_ast_modal(int sw, int sh){
    if(!ast_visible) return;

    DrawRectangle(0, 0, sw, sh, Fade(BLACK, 0.35f));
    int px = (int)roundf(80.0f * ui_scale);
    int py = (int)roundf(60.0f * ui_scale);
    int pw = sw - (int)roundf(160.0f * ui_scale);
    int ph = sh - (int)roundf(120.0f * ui_scale);
    if(pw < 420) pw = sw - 40;
    if(ph < 260) ph = sh - 40;

    DrawRectangleRounded((Rectangle){(float)px,(float)py,(float)pw,(float)ph},0.03f,8,(Color){245,245,245,255});
    DrawRectangleRoundedLines((Rectangle){(float)px,(float)py,(float)pw,(float)ph},0.03f,8,2.0f,BLACK);
    draw_ui_text("AST Hierarchy", px + (int)roundf(18.0f*ui_scale), py + (int)roundf(12.0f*ui_scale), 24, BLACK);

    int close_w = (int)roundf(80.0f * ui_scale), close_h = (int)roundf(32.0f * ui_scale);
    int close_x = px + pw - close_w - (int)roundf(14.0f * ui_scale);
    int close_y = py + (int)roundf(10.0f * ui_scale);
    DrawRectangleRounded((Rectangle){(float)close_x,(float)close_y,(float)close_w,(float)close_h},0.2f,6,(Color){170,80,80,255});
    draw_ui_text("Close", close_x + (int)roundf(14.0f*ui_scale), close_y + (int)roundf(6.0f*ui_scale), 16, WHITE);

    /* Tree area (top) */
    int tree_x = px + (int)roundf(16.0f*ui_scale);
    int tree_y = py + (int)roundf(54.0f*ui_scale);
    int tree_w = pw - (int)roundf(32.0f*ui_scale);
    int tree_h = (int)roundf((float)ph * 0.62f);
    DrawRectangleLines(tree_x, tree_y, tree_w, tree_h, (Color){70,70,70,255});

    if(ast_node_n > 0){
        int leaf = 0;
        float dx = (tree_w - 60.0f) / (float)(ast_node_n > 1 ? ast_node_n - 1 : 1);
        if(dx < 44.0f) dx = 44.0f;
        float dy = fmaxf(44.0f, (tree_h - 70.0f) / 6.0f);
        ast_layout_node(0, &leaf, (float)tree_x + 30.0f, dx, (float)tree_y + 24.0f, dy);

        for(int i=0;i<ast_node_n;i++){
            int p = ast_nodes[i].parent;
            if(p>=0){
                DrawLineEx((Vector2){ast_nodes[p].x, ast_nodes[p].y+8.0f},
                           (Vector2){ast_nodes[i].x, ast_nodes[i].y-8.0f},
                           2.0f, BLACK);
            }
        }
        for(int i=0;i<ast_node_n;i++){
            Vector2 m = MeasureTextEx(ui_font, ast_nodes[i].label, 16.0f*ui_scale, 1.0f*ui_scale);
            int nw = (int)m.x + (int)roundf(10.0f*ui_scale);
            int nh = (int)m.y + (int)roundf(6.0f*ui_scale);
            int nx = (int)roundf(ast_nodes[i].x) - nw/2;
            int ny = (int)roundf(ast_nodes[i].y) - nh/2;
            DrawRectangle(nx, ny, nw, nh, WHITE);
            DrawRectangleLines(nx, ny, nw, nh, BLACK);
            draw_ui_text(ast_nodes[i].label, nx + (int)roundf(5.0f*ui_scale), ny + (int)roundf(2.0f*ui_scale), 16, BLACK);
        }
    } else {
        draw_ui_text("No AST yet.", tree_x + 10, tree_y + 10, 18, (Color){90,90,90,255});
    }

    /* Symbol table area (bottom-left style) */
    int tab_x = tree_x;
    int tab_y = tree_y + tree_h + (int)roundf(10.0f*ui_scale);
    int tab_w = (int)roundf((float)tree_w * 0.55f);
    int tab_h = py + ph - tab_y - (int)roundf(12.0f*ui_scale);
    if(tab_h > 60){
        DrawRectangle(tab_x, tab_y, tab_w, tab_h, WHITE);
        DrawRectangleLines(tab_x, tab_y, tab_w, tab_h, BLACK);
        draw_ui_text("Index",  tab_x + 10, tab_y + 6, 16, BLACK);
        draw_ui_text("Symbol", tab_x + (int)roundf(70.0f*ui_scale), tab_y + 6, 16, BLACK);
        draw_ui_text("Type",   tab_x + (int)roundf(220.0f*ui_scale), tab_y + 6, 16, BLACK);
        DrawLine(tab_x, tab_y + (int)roundf(28.0f*ui_scale), tab_x + tab_w, tab_y + (int)roundf(28.0f*ui_scale), BLACK);

        int row_h = (int)roundf(22.0f*ui_scale);
        int max_rows = (tab_h - (int)roundf(32.0f*ui_scale)) / row_h;
        if(max_rows < 0) max_rows = 0;
        int rows = ast_sym_n < max_rows ? ast_sym_n : max_rows;
        for(int i=0;i<rows;i++){
            int yy = tab_y + (int)roundf(32.0f*ui_scale) + i*row_h;
            char ibuf[16]; snprintf(ibuf, sizeof(ibuf), "%d", ast_symbols[i].index);
            draw_ui_text(ibuf, tab_x + 10, yy, 15, BLACK);
            draw_ui_text(ast_symbols[i].symbol, tab_x + (int)roundf(70.0f*ui_scale), yy, 15, BLACK);
            draw_ui_text(ast_symbols[i].type, tab_x + (int)roundf(220.0f*ui_scale), yy, 15, BLACK);
        }
    }
}

/* Message de feedback */
static char  feedback[128]    = "Tape une commande et appuie sur ENTREE";
static Color feedback_col;
static float feedback_timer   = 0.f;

/* Smoke */
#define SM_MAX 60
typedef struct { float x,y,z,vx,vy,vz,life,ml; } Smoke;
static Smoke smokes[SM_MAX];
static int   sm_n=0;
static float exhaust_t=0.f;

/* ═══ HELPERS ═══════════════════════════════════ */
static const char *DIR_SYM[]  = {"↑ N","→ E","↓ S","← O"};
static const char *DIR_FULL[] = {"Nord","Est","Sud","Ouest"};
static int DX[]={0,1,0,-1};
static int DY[]={1,0,-1,0};

static float dir_ang(int d){ float a[]={0,90,180,270}; return a[d&3]; }

static void exec_push(Commande c){
    int nx=(exec_tail+1)%EXEC_MAX;
    if(nx!=exec_head){ exec_q[exec_tail]=c; exec_tail=nx; }
}
static int exec_pop(Commande *c){
    if(exec_head==exec_tail) return 0;
    *c=exec_q[exec_head]; exec_head=(exec_head+1)%EXEC_MAX; return 1;
}

static void smoke_emit(float x,float y,float z){
    if(sm_n<SM_MAX) smokes[sm_n++]=(Smoke){
        x,y,z,
        ((float)GetRandomValue(-12,12))/120.f,
        ((float)GetRandomValue(8,20))/100.f,
        ((float)GetRandomValue(-12,12))/120.f,
        0.f,1.4f};
}
static void smoke_upd(float dt){
    for(int i=0;i<sm_n;i++){
        smokes[i].x+=smokes[i].vx*dt; smokes[i].y+=smokes[i].vy*dt;
        smokes[i].z+=smokes[i].vz*dt; smokes[i].life+=dt;
        if(smokes[i].life>=smokes[i].ml){smokes[i]=smokes[--sm_n];i--;}
    }
}
static void smoke_draw(){
    for(int i=0;i<sm_n;i++){
        float t=smokes[i].life/smokes[i].ml;
        DrawSphere((Vector3){smokes[i].x,smokes[i].y,smokes[i].z},
                   0.04f+t*0.22f, Fade((Color){170,170,170,255},(1-t)*0.45f));
    }
}

/* Apply a command to the logical state */
static EtatRobot apply(EtatRobot e, Commande c){
    switch(c.type){
    case CMD_AVANCER:{
        int nx=e.x+DX[e.direction]*c.valeur;
        int ny=e.y+DY[e.direction]*c.valeur;
        if(nx>=0&&nx<e.grille_w&&ny>=0&&ny<e.grille_h){e.x=nx;e.y=ny;}
        else e.erreur=1; break;}
    case CMD_RECULER:{
        int nx=e.x-DX[e.direction]*c.valeur;
        int ny=e.y-DY[e.direction]*c.valeur;
        if(nx>=0&&nx<e.grille_w&&ny>=0&&ny<e.grille_h){e.x=nx;e.y=ny;}
        else e.erreur=1; break;}
    case CMD_TOURNER_DROITE: e.direction=(e.direction+1)%4; break;
    case CMD_TOURNER_GAUCHE: e.direction=(e.direction+3)%4; break;
    case CMD_DEMITOUR:       e.direction=(e.direction+2)%4; break;
    default: break;
    }
    return e;
}

static void start_next(){
    Commande c;
    if(!exec_pop(&c)){ animating=0; return; }
    if(c.type==CMD_RESET){
        /* Immediate reset */
        cur=etat_initial; cur.erreur=0;
        rob_x=tgt_x=etat_initial.x*CELL;
        rob_z=tgt_z=etat_initial.y*CELL;
        rob_angle=tgt_angle=dir_ang(etat_initial.direction);
        animating=0; path_n=0; sm_n=0;
        path[path_n++]=(P2){rob_x,rob_z};
        start_next(); return;
    }
        /* Compute next_state but do not commit it to `cur` until the animation finishes. */
        EtatRobot nxt = apply(cur, c);
        next_state = nxt;
        tgt_x = nxt.x * CELL; tgt_z = nxt.y * CELL;
        tgt_angle = dir_ang(nxt.direction);
        runtime_log("[INF][RUN][%d] start | type=%d val=%d target=(%d,%d) world=(%.2f,%.2f)",
            log_trace_get(),
            c.type, c.valeur, nxt.x, nxt.y, tgt_x, tgt_z);
    while(tgt_angle-rob_angle> 180.f) tgt_angle-=360.f;
    while(rob_angle-tgt_angle> 180.f) tgt_angle+=360.f;
     /* Keep `cur` as the logical state until movement completes
         so the HUD shows the current position while the animation runs. */
    animating = 1;
    if(path_n<MAX_PATH) path[path_n++]=(P2){rob_x,rob_z};
}

/* ═══ PARSING A COMMAND ═════════════════════════ */
static void set_feedback(const char *msg, Color col, float dur){
    strncpy(feedback,msg,127);
    feedback_col=col;
    feedback_timer=dur;
}

static void submit_command(){
    if(input_len==0) return;

    /* Convert to lowercase. */
    char cmd[INPUT_MAX];
    for(int i=0;i<=input_len;i++) cmd[i]=tolower((unsigned char)input_buf[i]);

    /* Add to history. */
    if(hist_n<HIST_MAX){
        strncpy(hist[hist_n].text, input_buf, INPUT_MAX-1);
        hist[hist_n].ok=0;
        hist_n++;
    } else {
        for(int i=0;i<HIST_MAX-1;i++) hist[i]=hist[i+1];
        strncpy(hist[HIST_MAX-1].text, input_buf, INPUT_MAX-1);
        hist[HIST_MAX-1].ok=0;
    }
    int trace_id = log_trace_next();
    log_trace_set(trace_id);
    runtime_log("[INF][SYN][%d] submit | input='%s'", trace_id, cmd);

    /* Parser */
    semantic_set_state(&cur);
    int ok = parse_command(cmd);

    if(ok && parsed_n>0){
        hist[hist_n>0?hist_n-1:0].ok=1;
        ast_build_from_parsed(input_buf);
        /* Enqueue all parsed commands. */
        for(int i=0;i<parsed_n;i++) exec_push(parsed_cmds[i]);
        char msg[128];
        if(parsed_n==1) snprintf(msg,127,"OK : %s", input_buf);
        else snprintf(msg,127,"OK : %s  (%d actions)", input_buf, parsed_n);
        set_feedback(msg, C_OK, 3.f);
        runtime_log("[OK][SYN][%d] parsed | actions=%d", trace_id, parsed_n);
    } else {
        char msg[128];
        /* Prefer parse_err if available. */
        if(parse_err[0]) snprintf(msg,127,"%s", parse_err);
        else snprintf(msg,127,"Erreur : commande inconnue \"%s\"", input_buf);
        set_feedback(msg, C_ERR, 4.f);
        runtime_log("[ERR][SYN][%d] %s", trace_id, msg);
    }

    /* Clear the field. */
    input_buf[0]='\0'; input_len=0;

    /* Start animation if idle. */
    if(!animating) start_next();
}

/* Batch execution in two passes: full validation first, then enqueue only if everything is valid. */
static void run_block(){
    if (batch_len == 0) { runtime_log("[INF][RUN][-] batch empty"); return; }

    char tmp[BATCH_MAX];
    strncpy(tmp, batch_buf, BATCH_MAX-1);
    tmp[BATCH_MAX-1] = '\0';

    /* First pass: validate each line without starting any animation. */
    int total_valid = 0, errors = 0;
    EtatRobot sem_state = cur;

    /* Accumulate validated commands to know how many will be enqueued later. */
    Commande  all_cmds[512];
    int       all_n = 0;

    char *saveptr = 0;
    for (char *line = strtok_r(tmp, "\n", &saveptr);
         line; line = strtok_r(NULL, "\n", &saveptr)) {

        /* Trim leading and trailing whitespace. */
        while (*line && isspace((unsigned char)*line)) line++;
        char *end = line + strlen(line) - 1;
        while (end >= line && isspace((unsigned char)*end)) *end-- = '\0';
        if (*line == '\0') continue;

        char cmd[INPUT_MAX]; int i = 0;
        for (; i < INPUT_MAX-1 && line[i]; i++) cmd[i] = tolower((unsigned char)line[i]);
        cmd[i] = '\0';

        int trace_id = log_trace_next();
        log_trace_set(trace_id);
        runtime_log("[INF][SYN][%d] batch validate | input='%s'", trace_id, cmd);

        /* Semantic validation uses the simulated state accumulated so far. */
        semantic_set_state(&sem_state);
        int ok = parse_command(cmd);

        if (ok && parsed_n > 0) {
            runtime_log("[OK][SYN][%d] valid | actions=%d", trace_id, parsed_n);
            for (int k = 0; k < parsed_n; k++) {
                if (all_n < 512) all_cmds[all_n++] = parsed_cmds[k];
                sem_state = apply(sem_state, parsed_cmds[k]);
            }
            total_valid++;
        } else {
            errors++;
            runtime_log("[ERR][SYN][%d] INVALID line → batch REJECTED | %s",
                        trace_id, parse_err[0] ? parse_err : "syntax error");
        }
    }

    /* All-or-nothing rule: a single error invalidates the whole block. */
    if (errors > 0) {
        char msg[128];
        snprintf(msg, 127, "Batch rejected: %d invalid line(s) — no commands executed", errors);
        set_feedback(msg, C_ERR, 5.f);
        runtime_log("[ERR][RUN][-] batch REJECTED | errors=%d valid=%d", errors, total_valid);
        return;   /* Do not touch the execution queue. */
    }

    /* Second pass: rebuild the AST and enqueue the commands. */
    ast_node_n = 0; ast_sym_n = 0;  /* Reset the AST for the batch. */

    /* Re-read the buffer so we reuse exactly the same lines as during validation. */
    strncpy(tmp, batch_buf, BATCH_MAX-1);
    tmp[BATCH_MAX-1] = '\0';
    saveptr = 0;
    sem_state = cur;

    for (char *line = strtok_r(tmp, "\n", &saveptr);
         line; line = strtok_r(NULL, "\n", &saveptr)) {

        while (*line && isspace((unsigned char)*line)) line++;
        char *end = line + strlen(line) - 1;
        while (end >= line && isspace((unsigned char)*end)) *end-- = '\0';
        if (*line == '\0') continue;

        char cmd[INPUT_MAX]; int i = 0;
        for (; i < INPUT_MAX-1 && line[i]; i++) cmd[i] = tolower((unsigned char)line[i]);
        cmd[i] = '\0';

        /* Re-parse to rebuild parsed_cmds before enqueueing. */
        semantic_set_state(&sem_state);
        parse_command(cmd);   /* Re-parse to rebuild parsed_cmds. */

        ast_accumulate_from_parsed(line);

        for (int k = 0; k < parsed_n; k++) {
            exec_push(parsed_cmds[k]);
            sem_state = apply(sem_state, parsed_cmds[k]);
        }
    }

    char msg[128];
    snprintf(msg, 127, "Batch OK : %d actions enqueuees", all_n);
    set_feedback(msg, C_OK, 3.f);
    runtime_log("[OK][RUN][-] batch enqueued | actions=%d", all_n);

    if (!animating) start_next();
}

/* ═══ ROBOT DRAWING ═════════════════════════════ */
static void draw_wheel(float wx,float wy,float wz){
    DrawCylinder((Vector3){wx,wy,wz},0.32f,0.32f,0.20f,14,C_WHL);
    DrawCylinderWires((Vector3){wx,wy,wz},0.32f,0.32f,0.20f,14,C_WHL2);
    DrawCylinder((Vector3){wx,wy,wz},0.15f,0.15f,0.22f,8,LIGHTGRAY);
}
static void draw_robot(float by){
    /* Chassis */
    DrawCube((Vector3){0,by+0.28f,0},1.7f,0.40f,2.6f,C_BOD2);
    DrawCube((Vector3){0,by+0.50f,-0.1f},1.6f,0.14f,2.3f,C_BOD);
    DrawCubeWires((Vector3){0,by+0.28f,0},1.7f,0.40f,2.6f,BLACK);
    /* Cabin */
    DrawCube((Vector3){0,by+0.76f,-0.20f},1.15f,0.38f,1.50f,C_CAB);
    DrawCubeWires((Vector3){0,by+0.76f,-0.20f},1.15f,0.38f,1.50f,C_ACC);
    /* Hood */
    DrawCube((Vector3){0,by+0.52f,0.90f},1.6f,0.12f,0.80f,C_BOD);
    /* Bumper */
    DrawCube((Vector3){0,by+0.22f,1.32f},1.55f,0.20f,0.10f,Fade(WHITE,0.15f));
    /* Headlights */
    DrawSphere((Vector3){-0.52f,by+0.36f,1.33f},0.10f,C_LDY);
    DrawSphere((Vector3){ 0.52f,by+0.36f,1.33f},0.10f,C_LDY);
    /* Rear lights */
    DrawSphere((Vector3){-0.52f,by+0.36f,-1.33f},0.09f,C_LDR);
    DrawSphere((Vector3){ 0.52f,by+0.36f,-1.33f},0.09f,C_LDR);
    /* Antenna */
    DrawCylinder((Vector3){0.38f,by+0.96f,-0.80f},0.022f,0.022f,0.60f,8,DARKGRAY);
    DrawSphere((Vector3){0.38f,by+1.56f,-0.80f},0.075f,C_LDG);
    /* Wheels */
    draw_wheel(-0.98f,by+0.18f, 0.85f);
    draw_wheel( 0.98f,by+0.18f, 0.85f);
    draw_wheel(-0.98f,by+0.18f,-0.85f);
    draw_wheel( 0.98f,by+0.18f,-0.85f);
}

/* ═══ GRID ══════════════════════════════════════ */
static void draw_grid(int w,int h){
    for(int i=0;i<w;i++) for(int j=0;j<h;j++){
        Color c=((i+j)%2==0)?C_GA:C_GB;
        DrawCube((Vector3){i*CELL,-0.07f,j*CELL},CELL,0.14f,CELL,c);
        DrawCubeWires((Vector3){i*CELL,-0.07f,j*CELL},CELL,0.14f,CELL,C_GL);
        char buf[16]; snprintf(buf,sizeof(buf),"%d,%d",i,j);
        /* DrawText3D may be unavailable in some raylib versions; skip 3D text labels. */
        (void)buf;
    }
    DrawCylinder((Vector3){0,0.08f,0},0.38f,0.38f,0.04f,16,C_ORG);
}

static void draw_path_3d(){
    for(int i=0;i<path_n;i++){
        DrawSphere((Vector3){path[i].x,0.13f,path[i].z},0.13f,C_PTH);
        if(i>0) DrawLine3D(
            (Vector3){path[i-1].x,0.13f,path[i-1].z},
            (Vector3){path[i  ].x,0.13f,path[i  ].z},C_PTH);
    }
    DrawCircle3D((Vector3){rob_x,0.16f,rob_z},.55f,(Vector3){0,1,0},0.f,Fade(C_ACC,.35f));
}

/* ═══ HUD ═══════════════════════════════════════ */
static void draw_hud(int sw, int sh, float time){
    float s = ui_scale;

    /* Left panel: robot state. */
    /* Left HUD: much larger. */
    int pw = (int)roundf(620.0f * s), ph = (int)roundf(520.0f * s);
    DrawRectangleRounded((Rectangle){8.0f*s,8.0f*s,(float)pw,(float)ph},0.10f,10,C_HUD);
    DrawRectangleRoundedLines((Rectangle){8.0f*s,8.0f*s,(float)pw,(float)ph},0.10f,10,3.0f,C_ACC);
        draw_ui_text("ROBOT 3D",    26,20,56,C_ACC);
        /* Zoom buttons (near the HUD top-right, stacked vertically). */
        int zb_w = (int)roundf(36.0f * s), zb_h = (int)roundf(36.0f * s);
        int zb_x = (int)roundf(8.0f*s) + pw - zb_w - (int)roundf(12.0f*s);
        int zb_y = (int)roundf(12.0f * s);
        int btn_gap = (int)roundf(8.0f * s);
        
        /* Zoom in button. */
        DrawRectangleRounded((Rectangle){(float)zb_x,(float)zb_y,(float)zb_w,(float)zb_h},0.12f,6,C_ACC);
        draw_ui_text("+", zb_x+10, zb_y+6, 24, BLACK);
        
        /* Zoom out button (below Zoom In). */
        int zout_y = zb_y + zb_h + btn_gap;
        DrawRectangleRounded((Rectangle){(float)zb_x,(float)zout_y,(float)zb_w,(float)zb_h},0.12f,6,C_ACC);
        draw_ui_text("-", zb_x+10, zout_y+6, 24, BLACK);
        
        /* Pause/resume rotation button (below Zoom Out). */
        int pause_y = zout_y + zb_h + btn_gap;
        Color pcol = grid_rotate_paused ? (Color){180,180,220,255} : C_ACC;
        DrawRectangleRounded((Rectangle){(float)zb_x,(float)pause_y,(float)zb_w,(float)zb_h},0.12f,6,pcol);
        /* Show the play icon when paused, and the pause icon when running. */
        draw_ui_text(grid_rotate_paused?"▶":"||", zb_x+8, pause_y+6, 20, BLACK);

        /* AST button (opens the AST hierarchy modal). */
        int ast_btn_y = pause_y + zb_h + btn_gap;
        Color ast_col = ast_visible ? (Color){0,220,140,255} : (Color){90,120,170,255};
        DrawRectangleRounded((Rectangle){(float)zb_x,(float)ast_btn_y,(float)zb_w,(float)zb_h},0.12f,6,ast_col);
        draw_ui_text("AST", zb_x+4, ast_btn_y+8, 14, BLACK);
    DrawLine((int)roundf(8.0f*s),(int)roundf(40.0f*s),(int)roundf(8.0f*s)+pw,(int)roundf(40.0f*s),C_ACC);
    char buf[80];
    /* Use measured text height to compute safe vertical spacing. */
    float base_x = 20.0f;
    int ycur = (int)roundf(64.0f * s);
    sprintf(buf,"Position  : (%d , %d)",cur.x,cur.y);
    draw_ui_text(buf,(int)base_x,ycur,22,WHITE);
    Vector2 m = MeasureTextEx(ui_font, buf, 22.0f * ui_scale, 1);
    int lh = (int)ceilf(m.y + 8.0f * ui_scale);
    ycur += lh;
    sprintf(buf,"Direction : %s",DIR_SYM[cur.direction]); draw_ui_text(buf,(int)base_x,ycur,22,WHITE);
    m = MeasureTextEx(ui_font, buf, 22.0f * ui_scale, 1);
    lh = (int)ceilf(m.y + 8.0f * ui_scale);
    ycur += lh;
    sprintf(buf,"Grille    : %dx%d",cur.grille_w,cur.grille_h); draw_ui_text(buf,(int)base_x,ycur,20,WHITE);
    m = MeasureTextEx(ui_font, buf, 20.0f * ui_scale, 1);
    lh = (int)ceilf(m.y + 8.0f * ui_scale);
    ycur += lh;
    int qlen=(exec_tail-exec_head+EXEC_MAX)%EXEC_MAX;
    sprintf(buf,"En attente: %d action(s)",qlen);
    draw_ui_text(buf,(int)base_x,ycur,20, qlen>0?(Color){255,200,0,255}:LIGHTGRAY);
    DrawLine((int)roundf(8.0f*s),(int)roundf(132.0f*s),(int)roundf(8.0f*s)+pw,(int)roundf(132.0f*s),(Color){50,50,90,255});
    /* Position the commands block just below the HUD status area. */
    int cmds_x = 20;
    int cmds_y = ycur + (int)roundf(28.0f * s);
    draw_ui_text("COMMANDES DISPONIBLES :",cmds_x,cmds_y,22,(Color){140,140,180,255});
    /* Compute per-line height for command entries. */
    Vector2 cmd_m = MeasureTextEx(ui_font, "avancer N    reculer N", 20.0f * ui_scale, 1);
    int cmd_lh = (int)ceilf(cmd_m.y + 8.0f * ui_scale);
    int ly = cmds_y + (int)roundf(36.0f * s);
    draw_ui_text("avancer N    reculer N",  cmds_x, ly, 20, LIGHTGRAY); ly += cmd_lh;
    draw_ui_text("tourner droite [N]",      cmds_x, ly, 20, LIGHTGRAY); ly += cmd_lh;
    draw_ui_text("tourner gauche [N]",     cmds_x, ly, 20, LIGHTGRAY); ly += cmd_lh;
    draw_ui_text("tourner demi-tour",      cmds_x, ly, 20, LIGHTGRAY); ly += cmd_lh;
    draw_ui_text("reset",                  cmds_x, ly, 20, LIGHTGRAY);
    if(cur.erreur)
        DrawText("HORS GRILLE !",20,ph-20,15,C_ERR);

    /* ── History (right panel) ── */
    /* Right history: much wider. */
    int hx = sw - (int)roundf(520.0f * s), hy = (int)roundf(8.0f * s), hw = (int)roundf(480.0f * s);
    /* Use measured text metrics for consistent line-height calculations. */
    Vector2 hist_m = MeasureTextEx(ui_font, "M", 20.0f * ui_scale, 1);
    int hist_line_h = (int)ceilf(hist_m.y + 8.0f * s);
    int hist_header_h = (int)roundf(44.0f * s);
    int hh = MIN(hist_n * hist_line_h + hist_header_h, sh/2);
    if(hist_n>0){
        DrawRectangleRounded((Rectangle){(float)hx,(float)hy,(float)hw,(float)hh},
                             0.08f,8,C_HUD);
        DrawRectangleRoundedLines((Rectangle){(float)hx,(float)hy,(float)hw,(float)hh},
                                  0.08f,8,1.5f,C_ACC);
        draw_ui_text("Historique",hx+18,hy+12,22,C_ACC);
        int max_visible = (hh - hist_header_h) / hist_line_h;
        if(max_visible < 1) max_visible = 1;
        int start = hist_n > max_visible ? hist_n - max_visible : 0;
        for(int i=start;i<hist_n;i++){
            int ly = hy + hist_header_h + (i-start) * hist_line_h;
            Color c = hist[i].ok?C_OK:C_ERR;
            draw_ui_text(hist[i].ok?"✓":"✗",hx+12,ly,20,c);
            draw_ui_text(hist[i].text,hx+60,ly,20,WHITE);
        }
    }

    /* ── Lexemes panel (under History) ── */
    {
        int lx = hx;
        int ly0 = hy + hh + (int)roundf(12.0f * s);
        int lw = hw;
        int lh = MIN(sh - ly0 - (int)roundf(120.0f*s), (int)roundf(260.0f*s));
        DrawRectangleRounded((Rectangle){(float)lx,(float)ly0,(float)lw,(float)lh},0.08f,8,C_HUD);
        DrawRectangleRoundedLines((Rectangle){(float)lx,(float)ly0,(float)lw,(float)lh},0.08f,8,1.5f,C_ACC);
        draw_ui_text("Lexemes (Flex)", lx+12, ly0+12, 20, C_ACC);

        /* Column positions. */
        float col_type_x = lx + 12.0f * s;
        float col_val_x  = lx + (int)roundf(120.0f * s);
        float col_line_x = lx + lw - (int)roundf(64.0f * s);

        /* Headers. */
        draw_ui_text("TYPE", (int)col_type_x, ly0 + (int)roundf(36.0f*s), 18, (Color){140,140,180,255});
        draw_ui_text("VALEUR", (int)col_val_x, ly0 + (int)roundf(36.0f*s), 18, (Color){140,140,180,255});
        draw_ui_text("LIGNE", (int)col_line_x, ly0 + (int)roundf(36.0f*s), 18, (Color){140,140,180,255});

        int lex_lines_h = lh - (int)roundf(56.0f*s);
        int lex_line_h = (int)roundf(20.0f * s);
        int lex_visible = lex_lines_h / lex_line_h;
        if(lex_visible < 1) lex_visible = 1;

        int start_lex = lex_n > lex_visible ? lex_n - lex_visible : 0;
        for(int i = start_lex; i < lex_n; i++){
            int row = i - start_lex;
            int yy = ly0 + (int)roundf(56.0f*s) + row * lex_line_h;
            draw_ui_text(lexemes[i].type, (int)col_type_x, yy, 18, LIGHTGRAY);
            draw_ui_text(lexemes[i].val,  (int)col_val_x,  yy, 18, WHITE);
            char lnbuf[16]; snprintf(lnbuf,sizeof lnbuf, "%d", lexemes[i].line);
            draw_ui_text(lnbuf, (int)col_line_x, yy, 18, (Color){180,180,200,255});
        }
        if(lex_n==0) draw_ui_text("Aucun lexeme capture.", lx+12, ly0 + (int)roundf(64.0f*s), 18, (Color){100,100,120,255});
    }

    /* ── Console (left, below HUD) ── */
    int console_x = (int)roundf(8.0f * s);
    int console_y = ph + (int)roundf(18.0f * s);
    int console_w = pw;
        int console_h = MIN(sh - console_y - (int)roundf(140.0f*s), (int)roundf(420.0f*s));
    DrawRectangleRounded((Rectangle){(float)console_x,(float)console_y,(float)console_w,(float)console_h},0.06f,6,C_HUD);
    DrawRectangleRoundedLines((Rectangle){(float)console_x,(float)console_y,(float)console_w,(float)console_h},0.06f,6,1.8f,C_ACC);
    draw_ui_text("Console", console_x+12, console_y+8, 20, C_ACC);
    int console_pad = (int)roundf(10.0f*s);
        int lines_h = console_h - (int)roundf(28.0f*s);
        int line_h = (int)roundf(24.0f*s);
    int lmax = MIN(ui_log_n, lines_h / line_h);
    for(int i=0;i<lmax;i++){
        int idx = ui_log_n - lmax + i;
            draw_ui_text(ui_log_buf[idx], console_x + console_pad, console_y + (int)roundf(28.0f*s) + i*line_h, 20, (i==lmax-1)?C_ACC:LIGHTGRAY);
    }
    /* Scrollbar indicator. */
    if(ui_log_n > lmax){
        float sv = (float)lmax / (float)ui_log_n;
        int sb_h = (int)(console_h * sv);
        int sb_y = console_y + 28 + (console_h - 28 - sb_h);
        DrawRectangle(console_x+console_w-12, sb_y, 6, sb_h, Fade(C_ACC, 0.9f));
    }

    /* ── Command block (multiline) ── */
    int batch_x = console_x;
    int batch_y = console_y + console_h + (int)roundf(12.0f*s);
    int batch_w = console_w;
    int batch_h = MIN(sh - batch_y - (int)roundf(160.0f*s), (int)roundf(160.0f*s));
    DrawRectangleRounded((Rectangle){(float)batch_x,(float)batch_y,(float)batch_w,(float)batch_h},0.06f,6,C_HUD);
    DrawRectangleRoundedLines((Rectangle){(float)batch_x,(float)batch_y,(float)batch_w,(float)batch_h},0.06f,6,1.6f,C_ACC);
    draw_ui_text("Bloc de commandes", batch_x+12, batch_y+8, 20, C_ACC);
    /* Run button. */
    int run_w=(int)roundf(120.0f*s), run_h=(int)roundf(28.0f*s);
    int run_x = batch_x + batch_w - run_w - (int)roundf(12.0f*s);
    int run_y = batch_y + (int)roundf(8.0f*s);
    Color run_col = batch_focused?C_ACC:(Color){80,80,120,255};
    DrawRectangle(run_x, run_y, run_w, run_h, run_col);
    draw_ui_text("Executer le bloc", run_x+12, run_y+6, 16, BLACK);

    /* Draw batch lines. */
    int batch_pad=(int)roundf(10.0f * s);
    int blines_h = batch_h - (int)roundf(36.0f*s);
    int bline_h = (int)fmaxf((float)roundf(16.0f*s), 18.0f * s);
    int bmax = blines_h / bline_h;
    /* Split batch_buf into lines and draw the last visible lines. */
    char tmp[BATCH_MAX]; strncpy(tmp, batch_buf, BATCH_MAX-1); tmp[BATCH_MAX-1]='\0';
    int lines_count = 0; char *lines[512];
    char *sp=0; for(char *ln=strtok_r(tmp, "\n", &sp); ln; ln=strtok_r(NULL, "\n", &sp)){
        lines[lines_count++]=ln; if(lines_count>=512) break;
    }
    int start = lines_count - bmax; if(start<0) start=0;
    for(int i=start;i<lines_count;i++){
        int ly = batch_y + (int)roundf(36.0f*s) + (i-start)*bline_h;
        draw_ui_text(lines[i], batch_x + batch_pad, ly, 18, (i==lines_count-1)?WHITE:LIGHTGRAY);
    }

    /* ── Feedback ── */
    if(feedback_timer>0){
        int fw=(int)(MeasureTextEx(ui_font, feedback, 18 * s, 1).x) + (int)roundf(30.0f*s);
        int fx=sw/2-fw/2;
        DrawRectangleRounded((Rectangle){(float)fx,(float)(sh-(int)roundf(130.0f*s)),(float)fw,48.0f*s},
                     0.3f,8,C_HUD);
        draw_ui_text(feedback,fx+15,sh-(int)roundf(126.0f*s),20,feedback_col);
    }

    /* ── Input area (bottom) ── */
    /* Larger input box. */
    int iy=sh-(int)roundf(120.0f*s), iw=sw-(int)roundf(40.0f*s), ix=(int)roundf(20.0f*s);
    /* Background. */
    Color border_col = input_focused ? C_ACC : (Color){60,60,100,255};
    DrawRectangleRounded((Rectangle){(float)ix,(float)iy,(float)iw,44.0f*s},0.15f,8,C_INP);
    DrawRectangleRoundedLines((Rectangle){(float)ix,(float)iy,(float)iw,44.0f*s},0.15f,8,2.f,border_col);

    /* Prompt. */
    draw_ui_text(">", ix+24, iy+14, 42, C_ACC);

    /* Typed text. */
    char display[INPUT_MAX+2];
    strncpy(display, input_buf, INPUT_MAX);
    /* Blinking cursor. */
    if(input_focused && (int)(time*2)%2==0)
        strncat(display,"_",1);
    draw_ui_text(display, ix+60, iy+14, 36, WHITE);

    /* Hint if empty. */
    if(input_len==0 && !((int)(time*2)%2==0))
        draw_ui_text("Tape une commande  (ex: avancer 3)", ix+60,iy+14,26,(Color){80,80,120,255});

    draw_ui_text("[ENTREE] Executer   [ESC] Quitter",
             ix+iw-(int)roundf(520.0f*s), iy+(int)roundf(26.0f*s), 18, (Color){80,80,120,255});

    DrawFPS(sw-80,10);

    /* AST modal rendered on top. */
    draw_ast_modal(sw, sh);
}

/* ═══════════════════════════════════════════════
    MAIN
    ═══════════════════════════════════════════════ */
int main(void){
    /* Initialize the default grid. */
    etat_initial.x=0; etat_initial.y=0;
    etat_initial.direction=NORD;
    etat_initial.grille_w=10;
    etat_initial.grille_h=10;
    etat_initial.erreur=0;

    SetConfigFlags(FLAG_MSAA_4X_HINT|FLAG_WINDOW_RESIZABLE);
    InitWindow(SCR_W,SCR_H,"Robot 3D — Saisie de Commandes en Temps Reel");
    SetTargetFPS(60);

    /* Try to load a larger UI font for readability (fallback to the default). */
    const char *font_paths[] = {
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
        NULL };
    int font_loaded = 0;
    for(int i=0; font_paths[i]; i++){
        if(FileExists(font_paths[i])){
            ui_font = LoadFontEx(font_paths[i], 48, 0, 0);
            font_loaded = 1; break;
        }
    }
    if(!font_loaded) ui_font = GetFontDefault();
    /* Initialize the default ui_scale value (the user can zoom). */
    ui_scale = 1.6f;

    float cx=(etat_initial.grille_w-1)*CELL/2.f;
    float cz=(etat_initial.grille_h-1)*CELL/2.f;

    Camera3D cam={0};
    cam.position  =(Vector3){cx,18.f,cz+24.f};
    cam.target    =(Vector3){cx,0.f,cz};
    cam.up        =(Vector3){0.f,1.f,0.f};
    cam.fovy      =45.f;
    cam.projection=CAMERA_PERSPECTIVE;

    cur=etat_initial;
    rob_x=tgt_x=0.f; rob_z=tgt_z=0.f;
    rob_angle=tgt_angle=0.f;
    path[path_n++]=(P2){rob_x,rob_z};

    feedback_col=C_ACC;
    float time=0.f, cam_t=0.f;

    while(!WindowShouldClose()){
        float dt=GetFrameTime();
        time+=dt; if(!grid_rotate_paused) cam_t+=dt*3.5f;
        if(feedback_timer>0) feedback_timer-=dt;

        int sw=GetScreenWidth(), sh=GetScreenHeight();

        /* ════ KEYBOARD INPUT ════ */
        /* Printable characters are routed to the focused widget. */
        int ch;
        while((ch=GetCharPressed())!=0){
            if(ch>=32 && ch<127){
                if(batch_focused){
                    if(batch_len < BATCH_MAX-1){ batch_buf[batch_len++]=(char)ch; batch_buf[batch_len]='\0'; }
                } else {
                    if(input_len < INPUT_MAX-1){ input_buf[input_len++]=(char)ch; input_buf[input_len]='\0'; }
                }
            }
        }
        /* Backspace handling (debounced single press). */
        if(IsKeyPressed(KEY_BACKSPACE)){
            if(time - last_backspace_time > 0.12f){
                if(batch_focused){ if(batch_len>0){ batch_buf[--batch_len]='\0'; } }
                else { if(input_len>0){ input_buf[--input_len]='\0'; } }
                last_backspace_time = time;
            }
        }
        /* Enter: submit in input mode; insert a newline in batch mode. */
        if(IsKeyPressed(KEY_ENTER)||IsKeyPressed(KEY_KP_ENTER)){
            if(time - last_enter_time > 0.12f){
                if(input_focused) submit_command();
                else if(batch_focused){ if(batch_len < BATCH_MAX-1){ batch_buf[batch_len++]='\n'; batch_buf[batch_len]='\0'; } }
                last_enter_time = time;
            }
        }
        /* Ctrl+L -> quick reset. */
        if(IsKeyDown(KEY_LEFT_CONTROL)&&IsKeyPressed(KEY_L)){
            Commande c={CMD_RESET,0}; exec_push(c);
            if(!animating) start_next();
        }
        /* Focus shortcuts: Ctrl+B for batch, Ctrl+I for input. */
        if(IsKeyDown(KEY_LEFT_CONTROL) && IsKeyPressed(KEY_B)){ batch_focused=1; input_focused=0; ui_log("Focus: batch"); }
        if(IsKeyDown(KEY_LEFT_CONTROL) && IsKeyPressed(KEY_I)){ batch_focused=0; input_focused=1; ui_log("Focus: input"); }
        /* Run block: Ctrl+R. */
        if(IsKeyDown(KEY_LEFT_CONTROL) && IsKeyPressed(KEY_R)){ run_block(); }
        /* Zoom shortcuts: Ctrl + +/-. */
        if(IsKeyDown(KEY_LEFT_CONTROL) && (IsKeyPressed(KEY_KP_ADD) || IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_DECIMAL))){ ui_scale = fminf(ui_scale + 0.15f, 3.0f); ui_log("UI scale: %.2f", ui_scale); }
        if(IsKeyDown(KEY_LEFT_CONTROL) && (IsKeyPressed(KEY_KP_SUBTRACT) || IsKeyPressed(KEY_MINUS))){ ui_scale = fmaxf(ui_scale - 0.15f, 0.6f); ui_log("UI scale: %.2f", ui_scale); }
        
        /* Mouse interactions: clicking the Run button or focusing areas. */
        if(IsMouseButtonPressed(MOUSE_LEFT_BUTTON)){
            int mx=GetMouseX(), my=GetMouseY();
            /* Compute batch-area coordinates (mirrors draw_hud). */
            float s = ui_scale;
            /* Handle the AST modal close hitbox first. */
            if(ast_visible){
                int px = (int)roundf(80.0f * s);
                int py = (int)roundf(60.0f * s);
                int pw = sw - (int)roundf(160.0f * s);
                int ph = sh - (int)roundf(120.0f * s);
                if(pw < 420) pw = sw - 40;
                if(ph < 260) ph = sh - 40;
                int close_w = (int)roundf(80.0f * s), close_h = (int)roundf(32.0f * s);
                int close_x = px + pw - close_w - (int)roundf(14.0f * s);
                int close_y = py + (int)roundf(10.0f * s);
                if(mx>=close_x && mx<=close_x+close_w && my>=close_y && my<=close_y+close_h){
                    ast_visible = 0;
                    runtime_log("[INF][SYN][-] AST modal closed");
                    continue;
                }
            }
            int pw = (int)roundf(620.0f * s), ph = (int)roundf(520.0f * s);
            int console_x = (int)roundf(8.0f * s);
            int console_y = ph + (int)roundf(18.0f * s);
            int console_w = pw;
            int console_h = MIN(sh - console_y - (int)roundf(140.0f*s), (int)roundf(420.0f*s));
            int batch_x = console_x;
            int batch_y = console_y + console_h + (int)roundf(12.0f*s);
            int batch_w = console_w;
            int batch_h = MIN(sh - batch_y - (int)roundf(160.0f*s), (int)roundf(160.0f*s));
            int run_w = (int)roundf(120.0f*s), run_h = (int)roundf(28.0f*s);
            /* Zoom button areas (must mirror draw_hud and stay stacked vertically). */
            int zb_w = (int)roundf(36.0f * s), zb_h = (int)roundf(36.0f * s);
            int zb_x = (int)roundf(8.0f*s) + pw - zb_w - (int)roundf(12.0f*s);
            int zb_y = (int)roundf(12.0f * s);
            int btn_gap = (int)roundf(8.0f * s);
            /* Zoom in button position. */
            int z_in_x = zb_x, z_in_y = zb_y;
            /* Zoom out button position. */
            int z_out_x = zb_x, z_out_y = zb_y + zb_h + btn_gap;
            /* Pause button position. */
            int pause_x = zb_x, pause_y = z_out_y + zb_h + btn_gap;
            /* AST button position. */
            int ast_x = zb_x, ast_y = pause_y + zb_h + btn_gap;
            int run_x = batch_x + batch_w - run_w - (int)roundf(12.0f*s);
            int run_y = batch_y + (int)roundf(8.0f*s);
            /* Click Zoom In. */
            if(mx>=z_in_x && mx<=z_in_x+zb_w && my>=z_in_y && my<=z_in_y+zb_h){ ui_scale = fminf(ui_scale + 0.15f, 3.0f); ui_log("UI scale: %.2f", ui_scale); }
            /* Click Zoom Out. */
            if(mx>=z_out_x && mx<=z_out_x+zb_w && my>=z_out_y && my<=z_out_y+zb_h){ ui_scale = fmaxf(ui_scale - 0.15f, 0.6f); ui_log("UI scale: %.2f", ui_scale); }
            /* Click Pause/Resume. */
            if(mx>=pause_x && mx<=pause_x+zb_w && my>=pause_y && my<=pause_y+zb_h){ grid_rotate_paused = !grid_rotate_paused; ui_log("Grid rotation paused: %d", grid_rotate_paused); }
            /* Click AST. */
            if(mx>=ast_x && mx<=ast_x+zb_w && my>=ast_y && my<=ast_y+zb_h){ ast_visible = !ast_visible; runtime_log("[INF][SYN][-] AST modal %s", ast_visible?"opened":"closed"); }
            /* Click Run. */
            if(mx>=run_x && mx<=run_x+run_w && my>=run_y && my<=run_y+run_h){ run_block(); }
            /* Click in the batch area -> focus batch. */
            if(mx>=batch_x && mx<=batch_x+batch_w && my>=batch_y && my<=batch_y+batch_h){ batch_focused=1; input_focused=0; }
            /* Click in the input area -> focus input. */
            int iy = sh - (int)roundf(120.0f*s), iw = sw - (int)roundf(40.0f*s), ix = (int)roundf(20.0f*s);
            if(mx>=ix && mx<=ix+iw && my>=iy && my<=iy+(int)roundf(44.0f*s)){ input_focused=1; batch_focused=0; }
        }

        /* ════ ANIMATION ════ */
        float mspd=6.0f, tspd=200.f;
        if(animating){
            float dx=tgt_x-rob_x, dz=tgt_z-rob_z;
            float dist=sqrtf(dx*dx+dz*dz);
            if(dist>0.01f){
                float step=mspd*dt;
                if(step>=dist){rob_x=tgt_x;rob_z=tgt_z;}
                else{rob_x+=dx/dist*step;rob_z+=dz/dist*step;}
                wspin+=step*220.f;
                exhaust_t+=dt;
                if(exhaust_t>0.08f){
                    float a=rob_angle*DEG2RAD;
                    smoke_emit(
                        rob_x+cosf(a+PI)*1.35f-sinf(a+PI)*0.55f,
                        0.18f,
                        rob_z+sinf(a+PI)*1.35f+cosf(a+PI)*0.55f);
                    exhaust_t=0;
                }
            }
            float da=tgt_angle-rob_angle;
            if(fabsf(da)>0.5f){
                float s=tspd*dt;
                rob_angle+=(da>0?1.f:-1.f)*fminf(s,fabsf(da));
            } else rob_angle=tgt_angle;

                if(fabsf(tgt_x-rob_x)<0.01f&&
               fabsf(tgt_z-rob_z)<0.01f&&
               fabsf(tgt_angle-rob_angle)<0.5f){
                    animating=0;
                    runtime_log("[OK][RUN][%d] done | pos=(%d,%d)", log_trace_get(), next_state.x, next_state.y);
                    /* Commit the logical state now that the animation is finished. */
                    cur = next_state;
                    if(path_n<MAX_PATH) path[path_n++]=(P2){rob_x,rob_z};
                    start_next(); /* Next command is waiting. */
                }
        } else {
            start_next();
        }

        bob_t=animating?sinf(time*18.f)*0.012f:0.f;
        smoke_upd(dt);

        /* ════ CAMERA ════ */
        cam.position.x=cx+cosf(cam_t*DEG2RAD+0.9f)*24.f;
        cam.position.z=cz+sinf(cam_t*DEG2RAD+0.9f)*24.f;
        cam.position.y=15.f+sinf(cam_t*DEG2RAD*0.2f)*1.5f;
        cam.target=(Vector3){cx,0.f,cz};

        /* ════ RENDERING ════ */
        BeginDrawing();
        ClearBackground(C_BG);
        BeginMode3D(cam);
            draw_grid(etat_initial.grille_w,etat_initial.grille_h);
            draw_path_3d();
            smoke_draw();
            rlPushMatrix();
                rlTranslatef(rob_x,bob_t,rob_z);
                rlRotatef(rob_angle,0.f,1.f,0.f);
                draw_robot(0.f);
            rlPopMatrix();
        EndMode3D();
        draw_hud(sw,sh,time);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}
