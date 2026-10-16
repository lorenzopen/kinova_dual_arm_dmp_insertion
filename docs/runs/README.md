# Run sul robot vero

- `logs/`: tutti i CSV di `logs/` (insert, giri a vuoto, wrench a mani vuote), con `logs/INDEX.md`.
- `20261016_1211_F75_Tt1_s9007_riuscito/`: la T inserita, con README e cell.yaml usato.
- `20261016_1333_F75_Zt1_s9012_mezzo_successo/`: la Z entrata, pila mezza caduta.

Le bag (`ros2 bag record`, tutti i topic /left /right) sono troppo grandi per git (~900 MB,
una sola da 170 MB). Dal tuo PC:

```bash
rsync -av --progress nvidia@<ip-pc-robot>:/home/nvidia/lorenzo/home/lorenzo/bags/ ./bags_robot/
```

Per leggerle: `ros2 bag info bags_robot/sessione_1016_1211` (formato mcap), oppure in Python con
`rosbag2_py` (vedi la verifica della linea di base: FK con `ros_backend._FK` sul joint_states).
