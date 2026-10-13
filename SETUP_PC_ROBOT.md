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

Cosa fa: dalla home va piano alla prima posa, poi la traiettoria in 40 s **dritta davanti**
al braccio (x = 400 mm, y ~ 0, z da 800 a 250 mm sopra la base, flangia rivolta in avanti),
poi **resta ferma lì**. Ctrl-C: si ferma dov'è. Se la flangia resta indietro più di 4 cm si
ferma da solo. Solo pose: niente forze, niente SOCP.

Opzioni: `--duration 60` più lento, `--forward 0.45` più avanti, `--lift 0.10` più in alto.

Per spegnere il braccio: Ctrl-C nel terminale 1.

---

## D. Andare via senza tracce

In Claude `/logout`, poi da un terminale normale:

```bash
rm -rf ~/lorenzo
```

---

Più avanti (non oggi): rumore della forza, calibrazione dello scaffale (`calibrate_cell.py`),
URDF col tri-pad, `insert` a due bracci - vedi HANDOFF.md.
