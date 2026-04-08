# Progetto CoderBot – Sistemi Embedded

Questo progetto riguarda lo sviluppo di un sistema di controllo per la base mobile [CoderBot](https://www.coderbot.org/it/index.html), con l’obiettivo di far percorrere al robot traiettorie predefinite mantenendo una velocità controllata.

Il sistema integra:
- Controllo dei motori
- Odometria
- Controllo cartesiano
- Scheduling firm real-time con algoritmo EDF

Per una descrizione completa del progetto, fare riferimento alla relazione:  
[Relazione](https://fabiolpr.github.io/progetto-coderbot/relazione.pdf)

---

## Demo

### Percorso rettilineo di 30cm
[![Demo](https://img.youtube.com/vi/HvvxdS-zI1w/0.jpg)](https://www.youtube.com/watch?v=HvvxdS-zI1w)    
▶ Clicca per vedere il video

### Percorso a semicerchio con diametro di 46cm
[![Demo](https://img.youtube.com/vi/vhKof3f0g3k/0.jpg)](https://www.youtube.com/watch?v=vhKof3f0g3k)    
▶ Clicca per vedere il video

---

## Visualizzazione dei percorsi

È possibile visualizzare percorsi eseguiti dal robot tramite il seguente file:  
[Visualizza percorsi](https://fabiolpr.github.io/progetto-coderbot/visualizza_percorsi.html)

Aprire il file in un browser per osservare graficamente le traiettorie seguite.

Il percorso mostrato rappresenta la traiettoria stimata dal robot tramite odometria, non quella reale. La stima è basata esclusivamente sui dati provenienti dagli encoder incrementali delle ruote, ed è quindi soggetta ad errori cumulativi nel tempo.
## Compilazione ed esecuzione

Per compilare il progetto:
```
make
```

Per eseguire il programma:
```
sudo ./build/coderbot.exe
```

Per pulire la directory:
```
make clean
```
---

## Funzionalità principali

### Controllo dei motori
Il robot non può impostare direttamente una tensione analogica continua ai motori, ma utilizza PWM(Pulse Width Modulation) per regolare la tensione media applicata.

Un algoritmo proporzionale-integrale regola dinamicamente il duty cycle in base all’errore tra velocità desiderata ed effettiva per mantenere la velocità obbiettivo.

### Odometria
La posizione del robot viene stimata utilizzando i tick degli encoder incrementali delle ruote del robot, convertiti in millimetri tramite costanti sperimentali.

### Controllo cartesiano
Il robot segue una sequenza di punti nello spazio, correggendo continuamente la propria traiettoria in base all’errore angolare rispetto al target.

### Scheduling real-time
Il sistema utilizza l’algoritmo di scheduling EDF (Earliest Deadline First) per gestire task periodiche dando garanzie temporali sulla loro esecuzione.