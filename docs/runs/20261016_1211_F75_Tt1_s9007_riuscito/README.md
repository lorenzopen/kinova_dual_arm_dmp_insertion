# 16/10/2026 12:11 - F75_Tt1_s9007, prima T inserita sul robot vero

```bash
W=F75_Tt1_s9007 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.main insert $W --arm $R --slow 3
```

- Controller: `cart_k` 1000/1000/1000/20/20/20, `cart_d` 5/5/5/1/1/1, `gen3_bare.urdf`, friction e kalman on.
- Tool 70 mm (`pad_x` 0.040 + `pad_r` 0.030), molle nuove più rigide.
- `jam_scale: 1.8`, `wrench_baseline: logs/wrench_20261016_112659.csv` (la copia è qui).
- Nessun JAM. Forza laterale stimata (corretta) max 9.2 N sulla route, 4.9 N nel seat;
  stretta misurata mediana 8-10 N.
- Placement misurato a mano: ___ mm (da scrivere).

File:
- `F75_Tt1_s9007_c100_five_cut800_20261016_121150.csv`: l'insert, un tick ogni 10 ms.
- `wrench_20261016_112659.csv`: la linea di base del wrench (giro a vuoto, gap 0.10).
- `follow2_F75_Tt1_s9007_20261016_112735.csv`: lo stesso giro a vuoto, lato comandi.
- `cell.yaml`: la configurazione usata.
- Bag della sessione (non copiata, troppo grande): `~/lorenzo/bags/sessione_1016_1211`.
