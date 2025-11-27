#ifndef MOTOR
#define MOTOR

//-------------------------------------
//  COSTANTI GLOBALI
//-------------------------------------
#define NMOT    2       // numero di motori
// paramteri controllore
#define GAIN    2       // guadagno di deafult
#define MAXGAIN 5
#define MINGAIN 0.5
#define WZERO   5       // zero rete
#define WPOLO   200     // polo rete
// inidicizzazione dei motori pan e tilt della camera
#define PAN     0
#define TILT    1
// inidicizzazione modalita` di controllo
#define P_ONLY  0   // controllo solo proporzionale
#define P_NET   1   // controllo proporzionale con rete anticipatrice
//-------------------------------------
//  DEFINIZIONE STRUTTURE
//-------------------------------------
struct motor_param{
    float R;    // Ohm
    float J;    // N m s^2 / rad
    float b;    // N m s / rad
    float kt;   // V s / rad
    float kb;   // N m / A
};
struct controller_param{
    float gain;     // guadagno statico
    float wz;       // pulsazione zero rad/s
    float wp;       // pulsazione polo rad/s
};
// struttura che contiene i parametri dell`equazione all differenze per la legge di controllo
struct control_law{
    float K;
    float A;
    float B;
    float C;
};
//-------------------------------------
//  PROTOTIPI FUNZIONI
//-------------------------------------
void    motor_init(float);
void    controller_init(float);
float   controller(int, float, float, int);
float   motor(int,float);
float   encoder(int);
void    set_gain(float);

#endif