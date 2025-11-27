/*
    flyrand.h
    flyrand.c
    Il modulo implementa le funzioni per simulare un oggetto che si muove dullo schermo.
    L-oggetto si muove con tre traiettorie diverse:
        1. RANDOM MODE  - traiettoria casuale
        2. WAVE MODE    - traiettoria sinusoidale 
        3. MOUSE MODE   - posizione del mouse
*/
#include "flyrand.h"
#include <stdlib.h>
#include <allegro.h>
#include <math.h>
//-------------------------------------
//  DEFINIZIONE FUNZIONI
//-------------------------------------
void bounce_obj(struct stato *object, int x0, int xwall, int y0, int ywall){
    int outl, outr, outu, outd;

    outl = ((object->x - object->r) <= x0);
    outr = ((object->x + object->r) >= xwall);
    outu = ((object->y + object->r) >= ywall);
    outd = ((object->y - object->r) <= y0);

    if(outl) object->x = x0 + object->r;
    if(outr) object->x = xwall - object->r;
    if(outl || outr) object->alpha = (float)PI - object->alpha;

    if(outd) object->y = y0 + object->r;
    if(outu) object->y = ywall - object->r;
    if(outu || outd) object->alpha = -object->alpha;
}
void init_obj(struct stato *object, float x, float y, float r,int col, float v, float alpha, float Xamp){
    object -> c = col;
    object -> r = r;
    object -> x = x;
    object -> y = y;
    object -> v = v;
    object -> alpha = alpha;
    object -> Xamp = Xamp;
}
void init_obj_mouse(struct stato object){
    BITMAP      *pointer;
    const int   width  = 2 * object.r + 6;
    const int   heigth = 2 * object.r + 6;

    pointer = create_bitmap(width,heigth);
    clear_bitmap(pointer);
    circlefill  (pointer, width/2, heigth/2, object.r , object.c);
    circle      (pointer, width/2, heigth/2, object.r+4 , object.c);
    set_mouse_sprite(pointer);

}
void move_obj(struct stato *object, float dt, float maxDalpha, float Vamp, int mode, float f, int xaxis, int yaxis){
    float                   dalpha;
    static unsigned int     k;
    static float            x0;

    dalpha = frand(-maxDalpha, maxDalpha);
    object -> alpha += dalpha;
    object -> v = Vamp;

    switch(mode){
        case 0: // modalita` volo casuale
            if (k !=0 ) k = 0;
            object -> y = object -> y + object -> v*sin(object -> alpha)*dt;
            object -> x = object -> x + object -> v*cos(object -> alpha)*dt;
            break;
        case 1: // modalita` sinusoide
            if (k == 0) {
                x0 = object->x;
                k = k + 1;
            }
            else{
                //object -> x = x0 + object->v*sin(2*PI*FREQ*k*dt)/(2*PI*FREQ);
                object -> x = x0 + object->Xamp*sin(2*PI*f*k*dt);
                k = k + 1;
            }
            break;
        case 2: // modalita` mouse
            object -> y = yaxis - mouse_y;
            object -> x = xaxis + mouse_x;
            break;
        default: break;
    }
    
}
void draw_obj(struct stato object, BITMAP *btm, int X0, int Y0){
    circlefill(btm, X0 + object.x, (Y0-1 - object.y), object.r-3 , object.c);
    circle(btm, X0 + object.x, (Y0-1 - object.y), object.r , object.c);    
}
float frand(float min, float first){
    float r;
    r = rand()/(float)RAND_MAX;
    return (min + r*(first - min));
}