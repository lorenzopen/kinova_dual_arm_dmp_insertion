# Setup sul PC robot (NVIDIA, Ubuntu 24.04, ROS 2 Jazzy)

Il PC è **condiviso** col collega: si usa il suo stesso utente Linux, ma tutto ciò che è mio
sta in **una sola cartella, `~/lorenzo`**, che alla fine si cancella con un comando.

Regole:
- non toccare `~/ros2_ws` (il suo workspace) né la configurazione di rete;
- niente `pip install` fuori dal mio venv (il suo controller vuole numpy < 2);
- ROS 2 in `/opt/ros/jazzy` è di sistema: si usa e basta, non crea conflitti.

## Struttura

```
~/lorenzo/
├── env.sh      ← da attivare in ogni terminale
├── home/       ← "casa" finta: Claude, login, cache, log ROS
├── venv/       ← Python con cvxpy (vede numpy 1.26 e rclpy di sistema)
├── ws/src/kinova_dual_arm_dmp_insertion
└── bash_history
```

`env.sh` non è un utente nuovo e non cambia niente di permanente: **solo nel terminale dove
lo attivo**, `HOME` diventa `~/lorenzo/home`, così tutto quello che i programmi scrivono
finisce lì. Chiudo il terminale e non resta niente di attivo.

---

## 1. Cartella e script (una volta sola)

In un terminale normale:

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

Attivarlo (il prompt comincia con `(lorenzo)`):

```bash
source ~/lorenzo/env.sh
```

**Da qui in poi, ogni terminale nuovo: prima `source ~/lorenzo/env.sh`.**

## 2. Venv Python

```bash
python3 -m venv --system-site-packages "$LORENZO/venv" && source "$LORENZO/venv/bin/activate" && pip install "cvxpy<1.7" clarabel "numpy<2"
```

Controllo: deve stampare `1.26.x`.

```bash
python -c "import numpy, cvxpy, rclpy, pinocchio; print(numpy.__version__)"
```

## 3. Claude Code

```bash
curl -fsSL https://claude.ai/install.sh | bash
```

Si installa dentro `~/lorenzo/home`. Avvio: `claude`, poi login.
(Claude Desktop per Linux: verificare su claude.ai/download; Claude Code da terminale basta.)

## 4. Il pacchetto

**Con chiavetta USB.** Sul PC del lab (Windows), nella cartella del repo:

```bash
git archive -o kinova_dual_arm_dmp_insertion.zip --prefix=kinova_dual_arm_dmp_insertion/ HEAD
```

Sul PC robot (la chiavetta compare in `/media/<utente>/...`):

```bash
cd "$LORENZO/ws/src" && unzip /media/*/*/kinova_dual_arm_dmp_insertion.zip
```

**Oppure da GitHub** (prima fare il push dal PC del lab; se il repo è privato serve un token,
che resta nella casa finta):

```bash
cd "$LORENZO/ws/src" && git clone https://github.com/lorenzopen/kinova_dual_arm_dmp_insertion.git
```

## 5. Prova senza robot

```bash
cd "$LORENZO/ws/src/kinova_dual_arm_dmp_insertion" && python -m kinova_dual_arm_dmp_insertion.main check T_s7039m4
```

Deve stampare la portata (max ~826 mm) e i `cart_k` da usare al launch.
Niente colcon: si lancia sempre con `python -m` dal venv.

## 6. Bracci: lo stack del lab

In un terminale **normale, SENZA `env.sh`**, come fa il collega (accordarsi prima con lui):

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && ros2 launch kinova_control dual_cartesian.launch.py ee_frame:=end_effector_link
```

Prima volta: rigidezza di default (350 N/m). Sinistro = .108, destro = .107.
I due terminali si vedono via ROS (stesso `ROS_DOMAIN_ID`).

## 7. Rumore della forza (prima misura vera)

Nel terminale `(lorenzo)`, braccio fermo:

```bash
ros2 service type /left/zero_wrench
```

Se è `std_srvs/srv/Trigger`:

```bash
ros2 service call /left/zero_wrench std_srvs/srv/Trigger
```

```bash
ros2 topic echo /left/estimated_wrench --field wrench.force
```

Guardare ~10 s. Stessa cosa con `/right`.
**Se oscilla più di ~0.3 N** il seat by feel (soglia ~0.85 N, pezzo 0.145 kg) non regge:
serve l'ATI Mini40 (`wrench_source: ft`) o pezzi più pesanti.

## 8. Prima del primo `insert`

- **URDF**: copiare `gen3_p2_F_only.urdf` in `~/lorenzo/` con la massa del tri-pad al posto
  del 2F-85 (0.925 kg); passarla come `urdf_path` al launch E in `config/cell.yaml`.
  L'originale non si tocca. Smontare 2F-85 / LinkerHand: concordarlo col lab.
- **Calibrare `config/cell.yaml`**: `shelf_in_base` (per braccio), `tool_in_ee`,
  `tool:` (pad_x, pad_r, r_tri), `piece_mass` (pesare il pezzo).
- **Rigidezza**: salire a gradini fino ai `cart_k` stampati da `check`.
  Mai abbassare `max_torque`.
- **Scacchiera**: fessura tra le colonne 2.5 mm (come testato) o 3 mm? Se 3 mm, rifare
  il test di scala sui 3 mondi.

Poi:

```bash
python -m kinova_dual_arm_dmp_insertion.main insert T_s7039m4
```

Chiede due conferme da tastiera (presentare il pezzo, lasciarlo). Ctrl-C ferma lo stream.
Log in `logs/`. Il piazzamento si misura a mano (cella peggiore rispetto al target).

---

## Andare via senza lasciare tracce

1. In Claude: `/logout`
2. Da un terminale **normale** (senza `env.sh`):

```bash
rm -rf ~/lorenzo
```

3. Facoltativo: in `~/.bash_history` del collega restano solo i comandi scritti prima di
   `source ~/lorenzo/env.sh`; si tolgono a mano (non usare `history -c`, cancellerebbe anche
   la sua cronologia).

Attenzione: finché `~/lorenzo` esiste, chi usa lo stesso utente può leggere il mio login
(`home/.claude`). Fare logout a fine giornata.
