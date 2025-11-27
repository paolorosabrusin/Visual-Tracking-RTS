//----------------------------------------------------------------
/*
    --- PROGETTO REAL-TIME SYSTEM ---
    main.c: Simulazione di una pan-tilt camera per il Visual Tracking
    autore: Paolo Rosa Brusin (matricola 564685)
*/
//----------------------------------------------------------------

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
//----------------------------------------------------------------
//  LIBRERIE
//----------------------------------------------------------------
#include <stdlib.h>
#include <stdio.h>
#include <allegro.h>
#include <math.h>
#include "mypthlib.h"
#include "flyrand.h"
#include "motor.h"
//----------------------------------------------------------------
// COSTANTI GLOABALI
//----------------------------------------------------------------
#ifndef PI
#define PI          3.14159
#endif
// ----- garfica -----
#define XWIN        1080
#define YWIN        ((int)(XWIN*9/16))    // = 608
#define YDATA       30
#define YCOMM       250
#define XDATA       220             // distanza dei dati in px dal bordo destro dela finestra 
#define XWALL       (XWIN-XDATA-20)
#define YWALL       (YWIN-10)
#define X0          10
#define Y0          10
// ---- colori ----
#define BKG         makecol(0,0,0)
#define RED         makecol(255,0,0)
#define BLUE        makecol(0,0,255)
#define GREEN       makecol(0,255,0)
#define MY_BLUE     makecol(0,100,200)
#define MY_GREEN    makecol(20,255,100)
#define MY_ORANGE   makecol(200,150,0)
#define RIQUADRO    makecol(200,75,0)
#define GREY        makecol(100,100,100) 
// ----- task -----
#define P           40              // periodo fisso
#define NT          5               // numero di thread
// ----- oggetto -----
#define RADIUS      (int)8        
#define DALPHA      5*PI/180
#define DVAMP       5
#define VMAX        300
#define MAXDA       45*PI/180
#define XOBJ        100             // ampizza in modalita` wave di volo
#define FREQ_INIT   1
#define FREQ_INC    0.05              
// ----- camera -----
#define L           200             // dimensione dell'immagine catturata dalla telecamera = quadrato 200x200
#define OBJ_PLAN    3000            // distanza tra la telecamera e il piano su cui si muove l'oggetto
#define SIZEMIN     40              // dimensione minima della finstra su cui viene eseguita la ricerca
#define DELTASZ     40
#define NSTEP       ((int)(L/DELTASZ-1))         
#define AREA_OBJ    (2*PI*RADIUS*RADIUS)                                  
#define THRESHOLD   (int)0.1*AREA_OBJ    // numero pixel soglia riconoscimento oggetto 10% dell`oggetto
#define INC         0
#define DEC         1    
/*---------------------------------------------------------------------------------------------------------*/
/* ATTENZIONE: ANALYSIS e COMANDI_UTENTE e CHECK non devono essere attivati insieme per una questione di
    sovrapposizione nella grafica
/*---------------------------------------------------------------------------------------------------------*/
//#define CHECK
// Time Analysis
//#define ANALYSIS
// Visualizzazione dei comandi
#define COMANDI_UTENTE            // da attivare se si vogliono fare vedere i comandi sullo schermo

//----------------------------------------------------------------
// DEFINIZIONE STRUTTURE
//----------------------------------------------------------------
struct point{
    float x;
    float y;
};
struct user_param{
    int     end;
    float   Da;
    float   Vamp;
    float   Kc;
    int     obj_mode;
    float   freq;
    int     control_mode;
    int     winstep;
};
//----------------------------------------------------------------
// RISORSE CONDIVISE
//----------------------------------------------------------------
struct point            target  = {x: XWALL/2, y: YWIN/2};
struct point            current = {x: XWALL/2, y: YWIN/2};
struct stato            obj;
int                     window  = L;
struct user_param       user    = {end: 0, Da: 0, Vamp: VMAX/4, Kc: 2, obj_mode: RANDOM_MODE, freq: FREQ_INIT, control_mode: P_NET, winstep: NSTEP};
//----------------------------------------------------------------
// VARIABILI GLOBALI
//----------------------------------------------------------------
const float             d       = OBJ_PLAN; // distanza tra la telecamera e il piano su cui si muove l'oggetto
int                     lost    = 1;
BITMAP                  *photo;             // simula il frame acquisito dalla telecamera
//----------------------------------------------------------------
// VARIABILI SEMAFORICHE
//----------------------------------------------------------------
pthread_mutex_t         mux_tgt;    // mutex per target
pthread_mutex_t         mux_crt;    // mutex per current
pthread_mutex_t         mux_scn;    // mutex per screen
//----------------------------------------------------------------
// PROTOTIPI FUNZIONI AUSILIARE
//----------------------------------------------------------------
void    error_call(int);
char    get_scancode(void);
void    init(void);
// ---  per VT_task      ---
int     centroide(BITMAP *, int, float *, float *, int);
void    prediction(float, float, float *, float *, int);
void    get_frame(BITMAP *,float,float);
int     decrementa_win(int, int, int *);
int     incrementa_win(int, int, int *);
int     change_win(int, int, int, int, int *);
// ---  per MOTOR_task   ---
void    inverse_congruence(float, float, float *, float *);
void    congruence_equations(float, float , float *, float *);
// ---  stampa dei dati  ---
void    draw_win(struct point , float , BITMAP *, int);
void    stampa_lost(BITMAP *, int, int, int);
void    stampa_agganciato(BITMAP *, int, int, int);
void    stampa_miss(BITMAP *, int, int, int, int, int);
void    stampa_double(BITMAP *, char *,double, int, int, int);
void    stampa_intero(BITMAP *, char *,int, int, int, int);
void    stampa_wcet(BITMAP *, double, int, int, int, int);
void    stampa_comandi(BITMAP *);
void    stampa_control(BITMAP *, char *, int, int, int);
//----------------------------------------------------------------
// PROTOTIPI TASK
//----------------------------------------------------------------
void *GFX_task(void *);
void *MOTOR_task(void *);
void *OBJ_task(void *);
void *USER_task(void *);
void *VT_task(void *);
//----------------------------------------------------------------
//   MAIN
//----------------------------------------------------------------
int main(){
    set_CPU(1);
    int j;
    init();
    /*---------------------------------------------------------------------*/
    /* task_create(indice,  nome,    periodo,     deadline,     priorita') */
    /*---------------------------------------------------------------------*/
    task_create(3,  MOTOR_task, 8,     8,     30);
    task_create(0,  OBJ_task,   10,     10,     25);
    task_create(2,  GFX_task,   30,     30,     20);
    task_create(4,  VT_task,    40,     40,     15);
    task_create(1,  USER_task,  60,     60,     10);

    for(j=0; j<NT; j++){ wait_for_task_end(j);}    
    allegro_exit();
    // salvataggio su file dei dati
    #ifdef ANALYSIS
    save_data_analysis();
    #endif
    return 0;
}
//----------------------------------------------------------------
// DEFINIZIONE DEI TASK
//----------------------------------------------------------------
void    *OBJ_task(void *arg){
    int     i;
    float   dt;

    i = task_argument(arg);
    
    init_obj(&obj, (int)frand(0,XWALL), (int)frand(0,YWIN), RADIUS, RED, user.Vamp, 0, XOBJ);

    dt = (float)task_period(i)/(float)1000;
    set_activation(i);

    while(!user.end){
        wait_for_period(i);
        move_obj(&obj, dt, user.Da, user.Vamp, user.obj_mode, user.freq, 0, YWIN);
        bounce_obj(&obj, X0, XWALL, X0, YWALL);
        deadline_miss(i);
    }
}
void    *USER_task(void *arg){
    int     i;
    static  int     one = 0;
    char    scan;

    i = task_argument(arg);
    set_activation(i);
    
    while(!user.end){
        wait_for_period(i);
        scan = get_scancode();
        switch (scan){
            case KEY_SPACE:
                if (one >= 1) break;
                //task_create(0, OBJ_task, P, P, 30);
                one++;
                break;
            case KEY_ESC:
                user.end = 1;
                break;
            case KEY_A:
                user.Da = user.Da + DALPHA;
                if(user.Da > MAXDA) user.Da = MAXDA;
                break;
            case KEY_S:
                user.Da -= DALPHA;
                if(user.Da < -MAXDA) user.Da = -MAXDA;
                break;
            case KEY_V:
                user.Vamp = user.Vamp + DVAMP;
                if(user.Vamp > VMAX) user.Vamp = VMAX;
                break;
            case KEY_B:
                user.Vamp -= DVAMP;
                if(user.Vamp < 0) user.Vamp = 0;
                break;
            // controllore
            case KEY_K:
                user.Kc = user.Kc + 0.1;
                if(user.Kc > 3) user.Kc = 3;
                set_gain(user.Kc);
                break;
            case KEY_L:
                user.Kc -= 0.1;
                if(user.Kc < 0) user.Kc = 0;
                set_gain(user.Kc);
                break;
            case KEY_W:
                user.obj_mode = WAVE_MODE;
                break;
            case KEY_R:
                user.obj_mode = RANDOM_MODE;
                break;
            case KEY_M:
                user.obj_mode = MOUSE_MODE;
                break;
            case KEY_F:
                user.freq = user.freq + FREQ_INC;
                if (user.freq > 3) user.freq = 2;
                break;
            case KEY_G:
                user.freq = user.freq - FREQ_INC;
                if (user.freq < FREQ_INC) user.freq = FREQ_INC;
                break;
            case KEY_C:
                if (user.control_mode == P_NET) user.control_mode = P_ONLY;
                else user.control_mode = P_NET;
                break;
            case KEY_P:
                if (user.winstep < NSTEP) user.winstep++;
                break;
            case KEY_O:
                if (user.winstep > 0) user.winstep--;
                break;
            default: break;
        }

        deadline_miss(i);
    }
}
void    *GFX_task(void *arg){
    int i, k;
    BITMAP *dynBuff; // bitmap su cui disegnare
    i = task_argument(arg);
    // inizializzazione della bitmap su cui disegnare
    dynBuff = create_bitmap(SCREEN_W,SCREEN_H);
    clear_to_color(dynBuff, BKG);

    set_activation(i);
    while(!user.end){
        wait_for_period(i);

        rect(dynBuff, X0, Y0, XWALL, YWALL, BLUE);

        draw_obj(obj, dynBuff, 0, YWIN);

        // mutua esclusione current
        pthread_mutex_lock(&mux_crt);
        draw_win(current, window, dynBuff, GREEN);
        pthread_mutex_unlock(&mux_crt);

        // PARAMETRI UTENTE
        textout_ex(dynBuff, font, "PARAMETRI UTENTE",(XWIN-XDATA)+20,(YDATA + 10),RIQUADRO,BKG);
        rect(dynBuff, (XWIN-XDATA)-5, (YDATA + 180), (XWIN-5), (YDATA+25), RIQUADRO);
        stampa_double(dynBuff, "velocity: ",(double)obj.v, (XWIN-XDATA), (YDATA + 40), MY_ORANGE);
        stampa_double(dynBuff, "alpha [deg]: ",(double)user.Da*180/PI, (XWIN-XDATA), (YDATA + 60), MY_ORANGE);
        stampa_double(dynBuff, "gain: ",(double)user.Kc, (XWIN-XDATA), (YDATA + 80), MY_ORANGE);
        stampa_double(dynBuff, "frequency [Hz]: ",(double)user.freq, (XWIN-XDATA), (YDATA + 100), MY_ORANGE);
        stampa_control(dynBuff, "controller: ", (XWIN-XDATA), (YDATA + 120), MY_ORANGE);
        stampa_intero(dynBuff, "win step: ",user.winstep, (XWIN-XDATA), (YDATA + 140), MY_ORANGE);
        if(lost) stampa_lost(dynBuff,(XWIN-XDATA),(YDATA + 160), RED);
        else stampa_agganciato(dynBuff,(XWIN-XDATA),(YDATA + 160), MY_GREEN);

        #ifdef ANALYSIS
        // MONITORAGGIO
        textout_ex(dynBuff, font, "TIME ANALYSIS",(XWIN-XDATA)+10,(YDATA + 200),GREY,BKG);
        rect(dynBuff, (XWIN-XDATA)-5, YWIN-80, (XWIN-5), (YDATA + 200), GREY);
        // stampa del numero di deadlinemiss di ogni task
        for (k=0; k<NT; k++){
            stampa_miss(dynBuff, k,get_deadline_miss(k), (XWIN-XDATA), (YDATA + 210 + k*60), RED);
        }
        //stampa execution time C(i)
        for (k=0; k<NT; k++){
            stampa_wcet(dynBuff, get_WCET(k)/(double)1000000, k, (XWIN-XDATA), (YDATA + 220 + k*60), MY_ORANGE);
            stampa_double(dynBuff, "tictoc (ms): ",get_computational_time(k)/(double)1000000, (XWIN-XDATA), (YDATA + 230 + k*60), MY_ORANGE);
        }
        #endif
        /* --- VISUALIZZAZIONE COMANDI     --- */
        #ifdef COMANDI_UTENTE
        stampa_comandi(dynBuff);
        #endif
        /*-------------------------------------*/
        /* --- CHECK VARIABILI DI CONTROLLO    --- */
        #ifdef CHECK
        stampa_intero(dynBuff, "target x: ",(int)target.x,XWALL+10,YWIN-100, MY_ORANGE);
        stampa_intero(dynBuff, "target y: ",(int)target.y,XWALL+10,YWIN-80, MY_ORANGE);
        stampa_intero(dynBuff, "current x: ",(int)current.x,XWALL+10,YWIN-60, MY_ORANGE);
        stampa_intero(dynBuff, "current y: ",(int)current.y,XWALL+10,YWIN-40, MY_ORANGE);
        stampa_intero(dynBuff, "win size: ",window, XWALL+10, YWIN-20, MY_ORANGE);
        #endif
        /*-------------------------------------*/
        pthread_mutex_lock(&mux_scn);
        blit(dynBuff, screen, 0,0,0,0, dynBuff->w,dynBuff->h);
        pthread_mutex_unlock(&mux_scn);

        clear_to_color(dynBuff, BKG);
        
        deadline_miss(i);
    }

    destroy_bitmap(dynBuff);
}
void    *MOTOR_task(void *arg){
    int     i;
    float   pan_tg, tilt_tg;    // angoli target
    float   theta, phi;         // theta = pan e phi = tilt
    float   x_tg, y_tg;         // coordinate target
    float   xp, yp;             // puntattore camera sul piano focale
    float   v;                  // tensione di controllo al motore
    float   sampletime;

    i = task_argument(arg);
    sampletime = (float)task_period(i)/(float)1000; // secondi
    motor_init(sampletime);
    controller_init(sampletime);

    set_activation(i);
    while(!user.end){
        wait_for_period(i);
        //----------------------------------------------------
        pthread_mutex_lock(&mux_tgt);
        x_tg = target.x;
        y_tg = target.y;
        pthread_mutex_unlock(&mux_tgt);
        //----------------------------------------------------
        inverse_congruence(x_tg, y_tg, &pan_tg, &tilt_tg);
        v       = controller(PAN, pan_tg, encoder(PAN), user.control_mode);
        theta   = motor(PAN, v);
        v       = controller(TILT, tilt_tg, encoder(TILT), user.control_mode);
        phi     = motor(TILT, v);
        congruence_equations(theta, phi, &xp, &yp);
        //----------------------------------------------------
        // check sulle variabili di controllo
        if( xp >= (XWALL-SIZEMIN/2) )   xp = (XWALL - SIZEMIN/2);
        if( xp <= (X0+SIZEMIN/2) )      xp = (X0+SIZEMIN/2);
        if( yp >= (YWALL-SIZEMIN/2) )   yp = (YWALL-SIZEMIN/2);
        if( yp <= (Y0+SIZEMIN/2) )      yp = (Y0+SIZEMIN/2);
        //-----------------------------------------------------
        pthread_mutex_lock(&mux_crt);
        current.x = xp;
        current.y = yp;
        pthread_mutex_unlock(&mux_crt);
        //----------------------------------------------------
        deadline_miss(i);
    }
}
void    *VT_task(void *arg){
    int     i, find;
    float   Dx, Dy;
    float   xc, yc;     // coordinate centroide
    float   xtg, ytg;   // coordinate target predizione
    float   xp, yp;     // coordinate in cui punta la telecamera
    int     x0, y0;     // coordinate bitmap photo
    int     win = window;    // finestra in cui si esegue il calcolo del centroide

    i = task_argument(arg);
    set_activation(i);

    while(!user.end){

        wait_for_period(i);

        /*** SIMULATED CAMERA *****************************/
        pthread_mutex_lock(&mux_crt);
        xp = current.x;
        yp = current.y;
        pthread_mutex_unlock(&mux_crt);
        //   ACQUISIZIONE FRAME
        get_frame(photo,xp,yp);
        /**************************************************/

        find = centroide(photo, win, &Dx, &Dy, RED);
        
        if(find == 1){
            xc = xp + Dx;
            yc = yp + Dy;
            prediction(xc, yc, &xtg, &ytg, lost);
            // --- aggiorno il target ----------
            // check sulle variabili di controllo
            if( xtg >= XWALL ) xtg = XWALL-1;
            if( xtg <= 0 )     xtg = 1;
            if( ytg >= YWIN )  ytg = YWIN-1;
            if( ytg <= 0 )     ytg = 1;
            pthread_mutex_lock(&mux_tgt);
            target.x = xtg;
            target.y = ytg;
            pthread_mutex_unlock(&mux_tgt);
            //----------------------------------
            //win = decrementa_win(win, DELTASZ, &lost);
            win = change_win(DEC, win, DELTASZ, user.winstep, &lost);
        }
        else{
            //win = incrementa_win(win, DELTASZ, user.winstep, &lost);
            win = change_win(INC, win, DELTASZ, user.winstep, &lost);
            if( lost ==  1){
                pthread_mutex_lock(&mux_tgt);
                target.x = XWALL/2;
                target.y = YWIN/2;
                pthread_mutex_unlock(&mux_tgt);
            }
        }
        window = win;
        deadline_miss(i);
    }
    destroy_bitmap(photo);
}
//----------------------------------------------------------------
//  DEFINIZIONE FUNZIONI AUSILIARE
//----------------------------------------------------------------
void    init(void){
    allegro_init();
    set_color_depth(32);
    set_gfx_mode(GFX_AUTODETECT_WINDOWED,XWIN,YWIN,0,0);
    //set_gfx_mode(GFX_AUTODETECT,XWIN,YWIN,XWIN_2,YWIN_2);
    clear_to_color(screen, BKG);
    install_keyboard();
    install_mouse();
    enable_hardware_cursor();

    // --- INIZIALIZZAZIONE FOTOCAMERA --- //
    photo = create_bitmap(L,L);
    clear_bitmap(photo);
    /***************************************/

    // --- INIZIALIZZAZIONE DEI MUTEX --- //
    pthread_mutexattr_t     mux_att;
    pthread_mutexattr_init(&mux_att);
    pthread_mutexattr_setprotocol(&mux_att, PTHREAD_PRIO_PROTECT);

    pthread_mutexattr_setprioceiling(&mux_att, 30);
    pthread_mutex_init(&mux_tgt, &mux_att);

    pthread_mutexattr_setprioceiling(&mux_att, 30);
    pthread_mutex_init(&mux_crt, &mux_att);

    pthread_mutexattr_setprioceiling(&mux_att, 20);
    pthread_mutex_init(&mux_scn, &mux_att);

    pthread_mutexattr_destroy(&mux_att);

}
void    draw_win(struct point winp, float winsz, BITMAP *btm, int color){

    int     x, y;
    int     half;
    int     x1,x2,y1,y2;

    x = (int)winp.x;
    y = (int)winp.y;
    half = (int)(winsz/2);
    /*
    if( (x-size/2)<0 )          x = size/2;
    if( (y-size/2)<0 )          y = size/2;
    if( (x + size/2) > XWALL)   x = XWALL - size/2;
    if( (y + size/2) > YWIN)    y = YWIN - size/2;       
    */
    x1 = x - half;
    y1 = YWIN - y + half;

    x2 = x + half;
    y2 = YWIN - y - half;

    rect(btm,x1,y1,x2,y2,color);
}
char    get_scancode(void){
    if (keypressed()) return readkey() >> 8;
    else return 0;
}
//-----------------------------------------------------------------
// Funzioni per il Visual Tracking
//-----------------------------------------------------------------
int     centroide(BITMAP *imag, int size, float *xc, float *yc, int color){
    /*
    (xc,yc) sono le coordinate del centroide rispetto al centro della bitmap.
    imag e` quadrata di dimensioni LxL 
    */
    int         sample = 0;
    float       xavg = 0;
    float       yavg = 0;
    int         px0 = (int)((L-size)/2);  
    const int   tg_c = color;
    
    for(int x=px0; x<=(size+px0); x++){
        for(int y=px0; y<=(size+px0); y++){
            if(getpixel(imag,x,y) == tg_c){
                sample ++;
                xavg = xavg + (x - xavg)/sample;
                yavg = yavg + (y - yavg)/sample;
            }
        }
    }
    if(sample > THRESHOLD){
        *xc = xavg - L/2;
        *yc = L/2 - yavg;
        return 1;
    }
    else{
        *xc = 0;
        *yc = 0;
        return 0;
    }
}
void    prediction(float x, float y, float *xr, float *yr, int lost){
    static float oldx = 0;
    static float oldy = 0;

    if(lost == 1){
        oldx = x;
        oldy = y;
    }

    *xr = 2 * x - oldx;
    *yr = 2 * y - oldy;

    if((*xr < 0) || (*xr > XWALL)) *xr = x;
    if((*yr < 0) || (*yr > YWIN)) *yr = y;

    oldx = x;
    oldy = y; 
}
void    get_frame(BITMAP* photo, float xp, float yp){
    int x0 = (int)(xp - L/2);
    int y0 = YWIN - (int)yp - (int)L/2;
    
    // mutua esclusione di screen usato anche da GFX_task 
    pthread_mutex_lock(&mux_scn);
    blit(screen,photo,x0,y0,0,0, photo->w,photo->h);
    pthread_mutex_unlock(&mux_scn);

}
int     decrementa_win(int win, int deltasz, int *lost){
    if (win > SIZEMIN) win = win - deltasz;
    if (win <= SIZEMIN) *lost = 0;
    return win;
}
int     incrementa_win(int win, int deltasz, int *lost){
    if (win < L) {
        win = win + deltasz;
    }
    else {
        *lost = 1;
    }
    return win;
}
int     change_win(int FLAG, int win, int deltasz, int nmax, int *lost){
    static int n = 0;

    switch (FLAG){
        case INC:
            if ((win < L) && (n < nmax)) {
                win = win + deltasz;
                n++;
            }
            else {
                *lost = 1;
            }
            break;
        case DEC:
            if (win > SIZEMIN) {
                win = win - deltasz;
                if (n>0) n--;
            }
            if (win <= SIZEMIN) *lost = 0;
            break;
        default: 
            break;
    }
    return win;
}
//-----------------------------------------------------------------
// Funzioni per la stampa a video dei dati
//-----------------------------------------------------------------
void    stampa_lost(BITMAP *btm, int x, int y, int color){
    textout_ex(btm, font, "LOST",x,y,color,-1);
}
void    stampa_agganciato(BITMAP *btm, int x, int y, int color){
    textout_ex(btm, font, "AGGANCIATO",x,y,color,-1);
}
void    stampa_double(BITMAP *btm, char s[10],double numero, int x, int y, int color){
    char str[40];

    sprintf(str, "%s = %3.4f", s,numero);
    textout_ex(btm, font, str, x, y, -1, -1);
    textout_ex(btm, font, str, x, y, color, -1);
}
void    stampa_intero(BITMAP *btm, char s[10],int numero, int x, int y, int color){
    char str[40];
    sprintf(str, "%s = %d", s,numero);
    textout_ex(btm, font, str, x, y, -1, -1);
    textout_ex(btm, font, str, x, y, color, -1);
}
void    stampa_miss(BITMAP *btm, int task_idx,int numero, int x, int y, int color){
    char str[40];

    sprintf(str, "miss(%d) = %d", task_idx,numero);
    textout_ex(btm, font, str, x, y, -1, -1);
    textout_ex(btm, font, str, x, y, color, -1);
}
void    stampa_wcet(BITMAP *btm, double numero, int task_idx, int x, int y, int color){
    char str[40];

    sprintf(str, "C(%d) [ms] = %10.9f", task_idx, numero);
    textout_ex(btm, font, str, x, y, -1, -1);
    textout_ex(btm, font, str, x, y, color, -1);
}
void    stampa_comandi(BITMAP *btm){
    const int c = MY_BLUE;
    textout_ex(btm, font, "COMANDI DISPONIBILI", XWIN-XDATA+20, YCOMM, c, 0);
    textout_ex(btm, font, "ESC: uscita", XWIN-XDATA, 20+YCOMM, c, 0);
    textout_ex(btm, font, "W:   Wave", XWIN-XDATA, 40+YCOMM, c, 0);
    textout_ex(btm, font, "R:   Random", XWIN-XDATA, 60+YCOMM, c, 0);
    textout_ex(btm, font, "M:   Mouse", XWIN-XDATA, 80+YCOMM, c, 0);
    textout_ex(btm, font, "K:   + gain", XWIN-XDATA, 100+YCOMM, c, 0);
    textout_ex(btm, font, "L:   - gain", XWIN-XDATA, 120+YCOMM, c, 0);
    textout_ex(btm, font, "F:   + frequency", XWIN-XDATA, 140+YCOMM, c, 0);
    textout_ex(btm, font, "G:   - frequency", XWIN-XDATA, 160+YCOMM, c, 0);
    textout_ex(btm, font, "A:   + alpha", XWIN-XDATA, 180+YCOMM, c, 0);
    textout_ex(btm, font, "S:   - alpha", XWIN-XDATA, 200+YCOMM, c, 0);
    textout_ex(btm, font, "V:   + velocity", XWIN-XDATA, 220+YCOMM, c, 0);
    textout_ex(btm, font, "B:   - velocity", XWIN-XDATA, 240+YCOMM, c, 0);
    textout_ex(btm, font, "P:   + win step", XWIN-XDATA, 260+YCOMM, c, 0);
    textout_ex(btm, font, "O:   - win step", XWIN-XDATA, 280+YCOMM, c, 0);
    textout_ex(btm, font, "C:   control mode", XWIN-XDATA, 300+YCOMM, c, 0);
}
void    stampa_control(BITMAP *btm, char s[10], int x, int y, int color){
    char str[40];
    if (user.control_mode == P_NET) sprintf(str, "%s = %s", s, "P_NET");
    else sprintf(str, "%s = %s", s, "P_ONLY");
    textout_ex(btm, font, str, x, y, -1, -1);
    textout_ex(btm, font, str, x, y, color, -1);
}
//-----------------------------------------------------------------
// Funzioni per la cinematica diretta e inversa
//-----------------------------------------------------------------
void    inverse_congruence(float x_tg, float y_tg, float *pan_tg, float *tilt_tg){
    float w, h;
    w = XWALL/2;
    h = YWIN/2;

    *pan_tg = atan((w - x_tg)/d);
    *tilt_tg = atan((y_tg - h)*cos(*pan_tg)/d);
}
void    congruence_equations(float theta, float gamma, float *x, float *y){
    float w, h;
    w = XWALL/2;
    h = YWIN/2;

    *x = w - d * tan(theta);
    *y = h + d * tan(gamma)/cos(theta);
}