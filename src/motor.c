/*------------------------------------------------------------------------------------
    motor.h 
    motor.c: modulo che implementa le equazioni per gestire piu` motori.
    Il numero di motori e` definito dalla define NMOT in motor.h
    I motori sono considerati identitici i.e. con le stesse caratteristiche tecniche.
    I motori sono controllati da controllori identici
    Il controllore e` un proporzionale con rete attenuatrice o anticipatrice (define: WPOLO e WZERO)
------------------------------------------------------------------------------------*/
#include "motor.h"

#include <math.h>

//-------------------------------------
//  DEFINIZIONE VARIABILI GLOBALI
//-------------------------------------
/*
 static nel motor.c serve a incapsulare la variabile
 così che è  “privata” per questo modolo modulo.
 Significa che ha linkaggio interno ed è visibile solo qui.
 Nessun file può accedere anche se nel motor.h uso `extern`
*/
static struct   motor_param         pmot;
static struct   controller_param    pcon = {gain: GAIN, wz: WZERO, wp: WPOLO};
static struct   control_law         eqn;
// buffer per l`equazione alle differenze
static float    y[NMOT][3];
static float    u[NMOT][3];
static float    e[NMOT][2];
static float    Kp                  = 2;

// parametri per l'equazione del modello del motore DC
static float        p;
static float        A;
static float        B;
//-------------------------------------
//  DEFINIZIONE FUNZIONI
//-------------------------------------
void    motor_init(float sample_time){

    float   K, tau, Ts;

    pmot.R = 1;
    pmot.J = 0.0005;
    pmot.b = 0.0001;
    pmot.kt = 0.05;
    pmot.kb = 0.05;

    K   = pmot.kt/(pmot.R*pmot.b + pmot.kt*pmot.kb);
    tau = pmot.R*pmot.J/(pmot.R*pmot.b + pmot.kt*pmot.kb);
    Ts  = sample_time;

    p   = exp((double)(-Ts/tau));
    A   = K*(Ts - tau*(1-p));
    B   = K*(tau*(1-p) - p*Ts);

}
float   motor(int idx, float volt){
    // equazone del modello
    y[idx][0] = (1+p)*y[idx][1] - p*y[idx][2] + A*u[idx][1] + B*u[idx][2];

    y[idx][2] = y[idx][1];
    y[idx][1] = y[idx][0];
    u[idx][2] = u[idx][1];
    u[idx][1] = volt;
    return y[idx][0];
}
float   encoder(int idx){
    return y[idx][0];
}
void    controller_init(float sample_time){
    eqn.K = pcon.gain*pcon.wp/pcon.wz;
    eqn.A = (sample_time*pcon.wz - 2)/(sample_time*pcon.wp + 2);
    eqn.B = (sample_time*pcon.wz + 2)/(sample_time*pcon.wp + 2);
    eqn.C = (sample_time*pcon.wp - 2)/(sample_time*pcon.wp + 2);
}
float   controller(int idx, float yd, float y, int MODE){
    e[idx][0] = yd - y;
    switch (MODE){
        case P_ONLY:
            u[idx][0] = Kp*e[idx][0];
            break;
        case P_NET:
            u[idx][0] = eqn.K*(eqn.B*e[idx][0] + eqn.A*e[idx][1]) - eqn.C*u[idx][1];
            break;
        default:
            u[idx][0] = eqn.K*(eqn.B*e[idx][0] + eqn.A*e[idx][1]) - eqn.C*u[idx][1];
            break;
    }
    // u[idx][0] = eqn.K*(eqn.B*e[idx][0] + eqn.A*e[idx][1]) - eqn.C*u[idx][1];
    e[idx][1] = e[idx][0];
    return u[idx][0];
}
void    set_gain(float gain){
    if ((gain <= MAXGAIN) || (gain >= MINGAIN) ){
        pcon.gain = gain;
        eqn.K = pcon.gain*pcon.wp/pcon.wz;
    }
}
