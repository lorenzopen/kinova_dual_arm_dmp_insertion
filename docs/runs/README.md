# Run sul robot vero che hanno inserito il pezzo

- `20261016_1211_F75_Tt1_s9007_riuscito/`: la T inserita, nessun JAM. Con la linea di base del
  wrench usata (`wrench_20261016_112659.csv`), il giro a vuoto da cui viene e il cell.yaml.
- `20261016_1333_F75_Zt1_s9012_mezzo_successo/`: la Z entrata, pila mezza caduta. Forze NON
  corrette (base della T attiva, percorso lontano). Accanto, la base della Z registrata dopo
  (`wrench_20261016_134246_Z_senza_contatto.csv`) per correggerle offline.

Le bag sono troppo grandi per git. Dal tuo PC:

```bash
rsync -av --progress nvidia@<ip-pc-robot>:/home/nvidia/lorenzo/home/lorenzo/bags/sessione_1016_1211 nvidia@<ip-pc-robot>:/home/nvidia/lorenzo/home/lorenzo/bags/sessione_1016_1335 ./bags_robot/
```
