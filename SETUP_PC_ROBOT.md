# PC robot (NVIDIA, Ubuntu 24.04, ROS 2 Jazzy) - guida corta

Tutto ciò che è mio sta in `~/lorenzo` (PC condiviso: non toccare `~/ros2_ws` del collega).
Terminale "mio" = uno dove ho fatto `source ~/lorenzo/env.sh` (il prompt comincia con `(lorenzo)`).

---

## A. Una volta sola

In un terminale **normale**:

```bash
mkdir -p ~/lorenzo/home ~/lorenzo/ws/src && cat > ~/lorenzo/env.sh <<'EOF'
# source ~/lorenzo/env.sh  -> everything of Lorenzo's stays inside ~/lorenzo
LORENZO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export HOME="$LORENZO/home"
export HISTFILE="$LORENZO/bash_history"
export ROS_LOG_DIR="$LORENZO/ros_log"
export PATH="$HOME/.local/bin:$PATH"
source /opt/ros/jazzy/setup.bash
[ -d "$LORENZO/venv" ] && source "$LORENZO/venv/bin/activate"
cd "$LORENZO/ws/src"
PS1="(lorenzo) $PS1"
EOF
```

URDF del braccio **senza end effector**: una copia di quello del lab senza la massa del
2F-85 (0.925 kg). Senza questo il controller compensa ~9 N di peso che non c'è e il braccio
tende a salire. Sempre nel terminale normale; deve stampare `1`:

```bash
sed 's|<mass value="0.925" />|<mass value="0.0001" />|' ~/ros2_ws/src/kinova_controller_pkgs/kinova_control/urdfs/gen3_p2_F_only.urdf > ~/lorenzo/gen3_bare.urdf && grep -c 'mass value="0.0001"' ~/lorenzo/gen3_bare.urdf
```

Poi nel mio ambiente: venv e pacchetto.

```bash
source ~/lorenzo/env.sh
```

```bash
python3 -m venv --system-site-packages "$LORENZO/venv" && source "$LORENZO/venv/bin/activate" && pip install "cvxpy<1.7" clarabel "numpy<2"
```

```bash
git clone https://github.com/lorenzopen/kinova_dual_arm_dmp_insertion.git
```

---

## B. Ogni volta: aggiornare il pacchetto

```bash
source ~/lorenzo/env.sh && cd "$LORENZO/ws/src/kinova_dual_arm_dmp_insertion" && git pull
```

---

## C. Oggi: un braccio (.109), senza end effector, traiettoria DMP lenta

**Terminale 1 - il nodo del lab** (normale, SENZA `env.sh`). Si collega al braccio, lo porta
in home e lo tiene in impedenza:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run kinova_control kinova_controller.py --ip 192.180.0.109 --ros-args -r __ns:=/left -p initial_mode:=cartesian -p home_on_start:=true -p ee_frame:=end_effector_link -p urdf_path:=$HOME/lorenzo/gen3_bare.urdf -p friction_enable:=true -p friction_urdf_path:=$HOME/lorenzo/gen3_bare.urdf -p kalman_enable:=true -p cart_k:='[600.0, 600.0, 600.0, 20.0, 20.0, 20.0]' -p cart_d:='[5.0, 5.0, 5.0, 1.0, 1.0, 1.0]'
```

Aspettare che sia in home e fermo.

**Terminale 2 - il nostro script** (mio ambiente):

```bash
source ~/lorenzo/env.sh && cd "$LORENZO/ws/src/kinova_dual_arm_dmp_insertion"
```

1. Prova a vuoto (non muove niente, stampa il piano):

```bash
python -m kinova_dual_arm_dmp_insertion.follow_one_arm F75_Tt1_s9007 --routes c100_five_cut800 --arm left --dry
```

2. Movimento vero (chiede Enter prima di partire - mano sull'E-stop):

```bash
python -m kinova_dual_arm_dmp_insertion.follow_one_arm F75_Tt1_s9007 --routes c100_five_cut800 --arm left
```

Cosa fa: lo scaffale immaginario è **alla sua sinistra**. Dalla home va piano alla prima
posa (~40 s), poi la traiettoria in 40 s nel **piano xz** a y = 200 mm (x tra 360 e 415 mm,
z da 800 a 250 mm sopra la base), **flangia rivolta verso +y** (a sinistra), poi **resta
ferma lì**. Ctrl-C: si ferma dov'è. Se la flangia resta indietro più di 4 cm si ferma da
solo. Solo pose: niente forze, niente SOCP.

Opzioni: `--duration 60` più lento, `--speed 0.005` avvicinamento più lento,
`--forward 0.45` più avanti, `--side 0.25` più a sinistra, `--lift 0.10` più in alto.

Per spegnere il braccio: Ctrl-C nel terminale 1.

---

## E. Due bracci: destro .107, sinistro .200, a mani vuote

Basi parallele, il **sinistro a y = +60 cm** dal destro. Il destro guarda verso **+y** (a
sinistra), il sinistro verso **-y** (a destra); il pezzo (immaginario) sta a metà, nello
stesso piano xz di oggi (x 360..415 mm, z 800..250 mm, ogni flangia a 220 mm dalla sua base
in y). Un solo loop, un solo orologio: le due pose arrivano allo stesso istante.

**Terminale 1 - braccio destro** (normale, SENZA `env.sh`):

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run kinova_control kinova_controller.py --ip 192.180.0.107 --ros-args -r __ns:=/right -p initial_mode:=cartesian -p home_on_start:=true -p ee_frame:=end_effector_link -p urdf_path:=$HOME/lorenzo/gen3_bare.urdf -p friction_enable:=true -p friction_urdf_path:=$HOME/lorenzo/gen3_bare.urdf -p kalman_enable:=true -p cart_k:='[600.0, 600.0, 600.0, 20.0, 20.0, 20.0]' -p cart_d:='[5.0, 5.0, 5.0, 1.0, 1.0, 1.0]'
```

**Terminale 2 - braccio sinistro** (normale, SENZA `env.sh`):

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 run kinova_control kinova_controller.py --ip 192.180.0.200 --ros-args -r __ns:=/left -p initial_mode:=cartesian -p home_on_start:=true -p ee_frame:=end_effector_link -p urdf_path:=$HOME/lorenzo/gen3_bare.urdf -p friction_enable:=true -p friction_urdf_path:=$HOME/lorenzo/gen3_bare.urdf -p kalman_enable:=true -p cart_k:='[600.0, 600.0, 600.0, 20.0, 20.0, 20.0]' -p cart_d:='[5.0, 5.0, 5.0, 1.0, 1.0, 1.0]'
```

Aspettare che entrambi siano in home e fermi.

**Terminale 3 - il nostro script** (mio ambiente):

```bash
source ~/lorenzo/env.sh && cd "$LORENZO/ws/src/kinova_dual_arm_dmp_insertion"
```

1. Prova a vuoto (stampa dove va ogni flangia e la distanza flangia-flangia):

```bash
python -m kinova_dual_arm_dmp_insertion.follow_two_arms F75_Tt1_s9007 --dry
```

2. Primo giro vero, punte degli end effector a 20 cm l'una dall'altra (Enter prima di partire):

```bash
python -m kinova_dual_arm_dmp_insertion.follow_two_arms F75_Tt1_s9007 --gap 0.20
```

3. Poi si stringe per gradi: `--gap 0.15`, poi `--gap 0.10` (= la larghezza del pezzo, la presa vera).
   `--gap` è la distanza fra le due punte (le facce di spugna che si guardano), tenuta lungo
   tutta la rotta; il default è 0.20.

Se uno dei due resta indietro più di 4 cm si fermano **tutti e due**; Ctrl-C idem. Alla
fine stampa per braccio l'errore di inseguimento peggiore, il rumore della forza stimata e
quanto è cambiata la distanza flangia-flangia rispetto al comando (= quanto si stringerebbe o
lascerebbe il pezzo). Log in `logs/follow2_*.csv`.

Opzioni come per un braccio: `--duration`, `--speed`, `--forward`, `--lift`, `--stop-error`;
`--sep 0.62` se la distanza vera fra le basi non è 60 cm.

**I 5 esempi col kit (board disegnate, quote, tutti i comandi in ordine):
[docs/ESEMPI_5_PEZZI.md](docs/ESEMPI_5_PEZZI.md).**

## F. Due bracci: inserimento completo (SOCP, forza)

Il pezzo vero fra le mani. Prima il controllo senza robot:

```bash
python -m kinova_dual_arm_dmp_insertion.main check F75_Tt1_s9007 --arm c100_five_cut800 --sep 0.60 --slow 3
```

stampa anche il `cart_k` per cui la rotta è addestrata (`[1500, 1500, 1500, 8, 40, 8]`):
rilanciare i terminali 1 e 2 con quello (o salire per gradi da 600). Poi:

```bash
python -m kinova_dual_arm_dmp_insertion.main insert F75_Tt1_s9007 --arm c100_five_cut800 --sep 0.60 --slow 3
```

Sequenza: posa aperta -> Enter (tieni il pezzo fra le piastre) -> avvicinamento, stretta in
anello chiuso sulla forza misurata -> Enter (lascia il pezzo) -> imbocco, rotta, inserimento
lento, appoggio "a sentire", rilascio, su di 15 cm. `--slow 3` = tre volte più lento
dell'addestramento. Si ferma da solo sopra 60 N per mano o 4 cm di errore. Log in `logs/`.

---

## D. Andare via senza tracce

In Claude `/logout`, poi da un terminale normale:

```bash
rm -rf ~/lorenzo
```

---

Più avanti: calibrazione dello scaffale (`calibrate_cell.py`, poi `--cell` / senza `--sep`),
URDF col tri-pad - vedi HANDOFF.md.
