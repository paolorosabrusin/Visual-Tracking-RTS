# Visual-Tracking-RTS
Progettino dell'esame di Real Time System

# Dependencies
Serve la libreria allegro per la grafica
```sh
sudo apt install liballegro-dev liballegro5-dev
``` 

# Compile
```sh
make
```

# Run

Nota: usare SCHED_FIFO richiede attenzione perché un thread realtime può bloccare il sistema se programmato in modo errato.

- Il codice originale imposta politiche realtime (SCHED_FIFO) e priorità via pthread attributes. Per usare SCHED_FIFO e priorità real-time su Linux è richiesto l'accesso a privilegi elevati (root) o la capability cap_sys_nice. Se l'app non ha questi permessi pthread_create fallisce con EPERM (Operation not permitted).
- Senza permessi, il fallback crea i thread con gli attributi di default (nessuna priorità realtime) — questo evita l'errore ma potrebbe cambiare il comportamento temporale del sistema real-time.

il `main` va quini lanciato con permessi root:
```sh
sudo ./bin/main
```