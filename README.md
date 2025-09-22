# Ricostruzione 3D dei Punti da Legare nella Gabbiatrice

Questo repository contiene il codice e le risorse relative alla **ricostruzione tridimensionale** dei punti da legare all'interno di una **gabbia cilindrica in acciaio**, realizzata da una macchina gabbiatrice Schnell. Il progetto è parte di un sistema automatizzato per il riconoscimento e la legatura robotizzata dei punti di incrocio tra i tondini longitudinali e la spirale.

## Obiettivo

Sviluppare una pipeline in grado di:

- Elaborare **point cloud** acquisite da una camera **SICK Visionary T-Mini**.
- Rilevare automaticamente i **tondini orizzontali e verticali** all’interno della struttura.
- Calcolare le **intersezioni 3D** tra i tondini, individuando con precisione i punti di legatura.
- Fornire le **coordinate spaziali** necessarie per il controllo automatico della macchina di legatura.
