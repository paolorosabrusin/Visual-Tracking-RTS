#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <errno.h>
#include <sched.h>
#include <time.h>
#include <unistd.h>
#include "mypthlib.h"

//-------------------------------------
//  DEFINIZIONE VARIABILI GLOBALI
//-------------------------------------
/*
 static nel mypthlib.c serve a incapsulare la variabile
 così che è  “privata” per questo modolo modulo.
 Significa che ha linkaggio interno ed è visibile solo qui.
 Nessun file può accedere anche se nel mypthlib.h uso `extern`
*/
static struct       task_param tp[NTASK];
static pthread_t    tid[NTASK];
// used for time analysis
static struct       timing myclock[NTASK];
static long         C[NTASK];   // Computational time in nanosecondi
static long         exet[NTASK][MAX_COUNT];
static long         miss[NTASK][MAX_COUNT];

/*-----------------------------------------------
    TIME MANAGEMENT
-----------------------------------------------*/
void    time_copy(struct timespec *t, struct timespec tstatic){
    t->tv_sec = tstatic.tv_sec,
    t->tv_nsec = tstatic.tv_nsec;
}
void    time_add_ms(struct timespec *t, int ms){
    t->tv_sec += ms/1000;
    t->tv_nsec += (ms%1000)*1000000;
    if (t->tv_nsec >= 1000000000){
        t->tv_sec += 1;
        t->tv_nsec -= 1000000000;
    }
}
int     time_cmp(struct timespec t1, struct timespec t2){
    if (t1.tv_sec   > t2.tv_sec ) return 1;
    if (t1.tv_sec   < t2.tv_sec ) return -1;
    if (t1.tv_nsec  > t2.tv_nsec) return 1;
    if (t1.tv_nsec  < t2.tv_nsec) return -1;
    return 0;
}
long    time_diff(struct timespec tnow, struct timespec told){
    struct timespec tdiff;
    long diff_ns;

    tdiff.tv_sec = tnow.tv_sec - told.tv_sec;
    tdiff.tv_nsec = tnow.tv_nsec - told.tv_nsec;
    if((tdiff.tv_nsec < 0) && (tdiff.tv_sec > 0)){
        tdiff.tv_sec -= 1;
        tdiff.tv_nsec = 1000000000 - tdiff.tv_nsec;
    }
    diff_ns = tdiff.tv_nsec + tdiff.tv_sec*1000000000;
    return diff_ns; 
}
/*-----------------------------------------------
    UTILITY FOR TIME ANALYSIS
-----------------------------------------------*/
void    tic(int i){
    clock_gettime(CLOCK_MONOTONIC, &myclock[i].tic);
    myclock[i].count++;
    return;
}
int     toc(int i){
    long  secondi;
    long  nanosec;

    if (  myclock[i].count < 0 ) myclock[i].count = 0;
    if (  myclock[i].count == 0 ) return 1;
    clock_gettime(CLOCK_MONOTONIC, &myclock[i].toc);

    // calcolo tempo trascorso dall'ultimo tic chiamato
    secondi = myclock[i].toc.tv_sec - myclock[i].tic.tv_sec;
    nanosec = myclock[i].toc.tv_nsec - myclock[i].tic.tv_nsec;
    if ( (nanosec < 0) && (secondi >= 1) ) {
        secondi = secondi - 1;
        nanosec = nanosec + 1000000000;
    }

    C[i] = secondi*1000000000 + nanosec;
    update_wcet(i);
    if(myclock[i].count > 0) myclock[i].count--;

    return 0;
}
int     tictoc(int i){
    long     secondi;
    long  nanosec;
    if (  myclock[i].count < 0 ) myclock[i].count = 0;
    if ( myclock[i].count == 0 ){
        clock_gettime(CLOCK_MONOTONIC, &myclock[i].tic);
        myclock[i].count++;
        return 1;
    }
    
    clock_gettime(CLOCK_MONOTONIC, &myclock[i].toc);
 
    // calcolo tempo trascorso dall'ultimo tic chiamato
    secondi = myclock[i].toc.tv_sec - myclock[i].tic.tv_sec;
    nanosec = myclock[i].toc.tv_nsec - myclock[i].tic.tv_nsec;
    if ( (nanosec < 0) && (secondi >= 1) ) {
        secondi = secondi - 1;
        nanosec = nanosec + 1000000000;
    }
    C[i] = secondi*1000000000 + nanosec;

    update_wcet(i);

    if(myclock[i].count > 0) myclock[i].count--;
    
    return 0;
}
double  get_computational_time(int i){
    return C[i];
}
long    toc_txt(int i){
    long  secondi;
    long  nanosec;

    if (  myclock[i].count < 0 ) myclock[i].count = 0;
    if (  myclock[i].count == 0 ) return 1;
    clock_gettime(CLOCK_MONOTONIC, &myclock[i].toc);

    // calcolo tempo trascorso dall'ultimo tic chiamato
    secondi = myclock[i].toc.tv_sec - myclock[i].tic.tv_sec;
    nanosec = myclock[i].toc.tv_nsec - myclock[i].tic.tv_nsec;
    if ( (nanosec < 0) && (secondi >= 1) ) {
        secondi = secondi - 1;
        nanosec = nanosec + 1000000000;
    }

    C[i] = secondi*1000000000 + nanosec;
    update_wcet(i);
    if(myclock[i].count > 0) myclock[i].count--;

    return C[i];
}
/*----------------------------------------------
    THREAD MANAGEMENT
----------------------------------------------*/
int     task_create(int i, void *(*body)(void*), int per, int dln, int prio){
    
    pthread_attr_t myatt;
    struct sched_param mypar;
    int tret;

    tp[i].argument = i;
    tp[i].period = per;
    tp[i].deadline = dln;
    tp[i].priority = prio;
    tp[i].dmiss = 0;

    pthread_attr_init(&myatt);
    pthread_attr_setinheritsched(&myatt, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&myatt, SCHED_FIFO);
    /**************************************************************************/
    /***    Esecuziione su un unico processore  ***/
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset);
    if (pthread_attr_setaffinity_np(&myatt, sizeof(cpu_set_t),&cpuset) != 0){
        printf("errore in set_CPU of thread %d\n",i);
    }
    /**************************************************************************/

    mypar.sched_priority = prio;
    pthread_attr_setschedparam(&myatt, &mypar);
    
    tret = pthread_create(&tid[i], &myatt, body, (void*)(&tp[i]));
    if(tret != 0) {
        /* print a more informative error message */
        printf("errore nella creazione del thread %d: %s\n", i, strerror(tret));
        /* fallback: try creating the thread without realtime attributes (may avoid EPERM) */
        if (tret == EPERM) {
            printf("tentativo di creazione senza attributi realtime per il thread %d\n", i);
            /* try default creation without touching myatt */
            tret = pthread_create(&tid[i], NULL, body, (void*)(&tp[i]));
            if (tret != 0) {
                printf("fallback: errore nella creazione del thread %d: %s\n", i, strerror(tret));
            } else {
                printf("thread %d creato senza attributi realtime\n", i);
            }
        }
    }

    pthread_attr_destroy(&myatt);
    
    return tret;
}
int     task_argument(void *arg){
    struct task_param *myarg;
    myarg = arg;
    return myarg->argument;
}
void    set_activation(int i){
    /*  calcola il prossimo istante di attivazione,
        e imposta la prossima deadline assoluta.
    */

    struct timespec t;

    clock_gettime(CLOCK_MONOTONIC, &t);
    time_copy(&(tp[i].at), t);
    time_copy(&(tp[i].dl), t);
    time_add_ms(&(tp[i].at), tp[i].period);
    time_add_ms(&(tp[i].dl), tp[i].deadline);
    return;

}
int     task_period(int i){
    return tp[i].period;
}
int     wait_for_period(int i){
    struct  timespec    rem = {tv_sec: 0, tv_nsec: 0};
    time_t              myrem;
    int                 ret;
    int flag;
    // sospensione dell-esecuzione del task fino al prossimo istante di attivazione
    flag = save_exet(i);
    ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &(tp[i].at), &rem);  
    // QUANDO IL THREAD SI SVEGLIA NON VA SUBITO IN RUN !!
    tic(i);
    if(ret != 0){
        myrem =     rem.tv_sec*1000000000; // ns
        myrem +=    rem.tv_nsec;
        printf("il task %d non ha dormito bene. rem %d = %ld\n",i,i,myrem);
    }

    // update del prossimo istante di attivazione
    time_add_ms(&(tp[i].at), tp[i].period);
    time_add_ms(&(tp[i].dl), tp[i].period);
    return flag;
}
void    wait_for_task_end(int i){
    pthread_join(tid[i], NULL);
}
// ---------------------------------------------
//  handling deadline miss
// ---------------------------------------------
int     deadline_miss(int i){
    struct  timespec now;
    //double  delay = 0;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if ( time_cmp( now, tp[i].dl ) == 1 ){
        tp[i].dmiss++;
        save_miss(i,1);
        //-------------------------
        //time_add_ms(&(tp[i].at), tp[i].period);
        //time_add_ms(&(tp[i].dl), tp[i].period);
        // in questo modo salto l-esecuzione nel periodo corrente
        //-------------------------
        //delay = time_diff(now, tp[i].dl)/(double)1000000;
        //printf("latness %d>: %f\n",i,delay);
        return 1;
    }
    else{
        save_miss(i,0);
        return 0;
    } 
}
int     get_deadline_miss(int i){
    return tp[i].dmiss;
}
// ---------------------------------------------
//  handling Worst Case Execution Time
// ---------------------------------------------
void    update_wcet(int i){     // Inserita nell funzione toc() (definita sopra)
    if( C[i] > tp[i].WCET ) tp[i].WCET = C[i];
    return;
}
long    get_WCET(int i){
    return tp[i].WCET;
}
int     save_exet(int idx){

    static int count[NTASK];

    if(count[idx] < MAX_COUNT) {
        exet[idx][count[idx]] = toc_txt(idx);
        count[idx] = count[idx] + 1;
    }
    if (count[idx] >= MAX_COUNT) {
        return 1;
    }
    else {
        return 0;
    }
    return 0;

}
int     save_miss(int idx, int n){

    static int count[NTASK];

    if(count[idx] < MAX_COUNT) {
        miss[idx][count[idx]] = n;
        count[idx] = count[idx] + 1;
    }
    if (count[idx] >= MAX_COUNT) {
        return 1;
    }
    else {
        return 0;
    }
    return 0;

}
void    save_data_analysis(){
    /* SALVATAGGIO DATI */
    FILE *fp;
    if(!(fp = fopen("./dati_ct.txt","w"))){

        printf("file non corretto\n");
    }
    for(int j = 1; j<MAX_COUNT; j++){
        for(int s=0; s<(NTASK-1); s++){
            fprintf(fp, "%ld,", exet[s][j]);
        }
        fprintf(fp, "%ld\n", exet[NTASK-1][j]);
    }
    fclose(fp);

    /* SALVATAGGIO DEADLIINE MISS*/
    FILE *fp2;
    if(!(fp2 = fopen("./dati_miss.txt","w"))){

        printf("file non corretto\n");
    }
    for(int j = 0; j<MAX_COUNT; j++){
        for(int s=0; s<(NTASK-1); s++){
            fprintf(fp2, "%ld,", miss[s][j]);
        }
        fprintf(fp2, "%ld\n", miss[NTASK-1][j]);
    }
    fclose(fp2);
}
// ---------------------------------------------
//  CPU setting
// ---------------------------------------------
void set_CPU(int cpu){
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu, &cpuset);
    if (sched_setaffinity(0,sizeof(cpu_set_t),&cpuset) != 0){
        printf("errore in set_CPU\n");
    }
}