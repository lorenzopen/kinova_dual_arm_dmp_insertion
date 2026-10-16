# 16/10/2026 13:33 - F75_Zt1_s9012, MEZZO SUCCESSO

```bash
W=F75_Zt1_s9012 R=c100_five_cut800
python -m kinova_dual_arm_dmp_insertion.main insert $W --arm $R --slow 3
```

- **Esito: la Z è entrata, ma la pila è mezza caduta.**
- Controller come la T delle 12:11: `cart_k` 1000/1000/1000/20/20/20, `gen3_bare.urdf`.
- Tool 70 mm, `jam_scale: 1.8`. Nessun JAM.
- ATTENZIONE: `wrench_baseline` era ancora quella della T (`logs/wrench_20261016_112659.csv`),
  non una della Z: dove il percorso della Z se ne allontana più di 30 mm la forza non è corretta.
- Dal log: forza laterale stimata max 6.4 N (route e seat), forza verticale fino a 11.5 N
  sulla route e 6-9 N nel seat; stretta mediana 10.3 N sulla route, 5.6 N nel seat.
- Placement misurato a mano: ___ mm (da scrivere). Quali pezzi sono caduti: ___.

File: il log dell'insert e `cell.yaml` com'era. Bag: `~/lorenzo/bags/sessione_1016_1335`.

Controllo a posteriori: il percorso della Z passa a 160-195 mm dalla linea di base della T,
quindi la correzione era attiva sullo 0% dei tick: forze tutte grezze. Il seat "a sentire" è
durato 0 s con ext_z 6-9 N (la soglia è il 60% del peso, ~0.9 N): è possibile che abbia
creduto il pezzo appoggiato e l'abbia lasciato troppo presto. Prossima Z: prima la sua linea
di base (passo 7 di ESEMPI_5_PEZZI.md).
