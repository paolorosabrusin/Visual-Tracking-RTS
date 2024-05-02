#ifndef MYPTHLIB
#define MYPTHLIB

#define _GNU_SOURCE
#include <pthread.h>
#include <time.h>
#include <sched.h>
#include <unistd.h>

#ifndef NTASK
#define NTASK   5
#endif

#define MAX_COUNT   5000

struct task_param{
    int argument;
    long WCET;              // Worst Case Execution Time in ns
    int period;             // ms
    int deadline;           // deadline relativa ms
    int priority;
    int dmiss;
    struct timespec at;     // istante di attivazione
    struct timespec dl;     // deadline assoluta
};
/*-----------------------------------------------
    Utility for la Time analysis
-----------------------------------------------*/
struct timing{
    struct timespec tic;
    struct timespec toc;
    int count;
};

/*-----------------------------------------------
    TIME MANAGEMENT
-----------------------------------------------*/
int     time_cmp(struct timespec, struct timespec);
void    time_copy(struct timespec *, struct timespec);
void    time_add_ms(struct timespec *, int);
long    time_diff(struct timespec, struct timespec);
/*-----------------------------------------------
    TIME ANALYSIS
-----------------------------------------------*/
void    tic(int);
int     toc(int);
int     tictoc(int);
double  get_computational_time(int);
long    toc_txt(int);
/*-----------------------------------------------
    GESTIONE TASK PERIODICI
-----------------------------------------------*/
int     task_create(int, void *(void *), int, int, int);
int     task_argument(void *);
int     task_period(int);
int     wait_for_period(int);
void    wait_for_task_end(int);
void    set_activation(int);
//-----------------------------------------------
int     deadline_miss(int);
int     get_deadline_miss(int);
void    update_wcet(int);
long    get_WCET(int);
int     save_exet(int);
int     save_miss(int,int);
void    save_data_analysis();
//-----------------------------------------------
void set_CPU(int);

#endif