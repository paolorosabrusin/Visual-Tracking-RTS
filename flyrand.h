#ifndef FLYRAND
#define FLYRAND

#include <allegro.h>
//-------------------------------------
//  COSTANTI GLOBALI
//-------------------------------------
#define PI              3.14159
#define FREQ            0.5
#define RANDOM_MODE     (int)0     
#define WAVE_MODE       (int)1
#define MOUSE_MODE      (int)2
//-------------------------------------
//  DEFINIZIONE STRUTTURE
//-------------------------------------
struct stato{
    int c;
    float r;
    float x;
    float y;
    float v;
    float alpha;
    float Xamp;
};
//-------------------------------------
//  PROTOTIPI FUNZIONI
//-------------------------------------
void bounce_obj(struct stato *, int, int, int, int);
void move_obj(struct stato *, float, float, float, int,float, int, int);
void init_obj(struct stato *, float, float, float,int, float, float,float);
void draw_obj(struct stato, BITMAP *, int, int);
float frand(float,float);

#endif