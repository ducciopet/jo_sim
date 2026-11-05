#!/bin/bash
# Setup del PC del laboratorio (Ubuntu 24.04) per lavorare sui dati SINS sul NAS e comandarlo da remoto
# con Claude Code Remote Control. Si lancia UNA volta, da utente normale (chiede sudo quando serve):
#   bash setup_lab_pc.sh
# Si puo' rilanciare senza danni: ogni passo salta cio' che e' gia' fatto.
set -e
NAS_IP=192.168.178.111
NAS_SHARE="Kloster Reichenau"
NAS_USER="Lo Imperatore"
MNT=/mnt/nas-imperatore
REPO=https://github.com/ducciopet/jo_sim.git
BRANCH=sins
DIR=$HOME/Desktop/jo_sim_sins
ok(){ echo -e "\e[32m[OK]\e[0m $*"; }; step(){ echo -e "\n\e[1m== $*\e[0m"; }; warn(){ echo -e "\e[33m[!!]\e[0m $*"; }

step "1/8 Pacchetti di base"
sudo apt-get update -qq
sudo apt-get install -y -qq cifs-utils git tmux curl unzip ca-certificates >/dev/null
ok "cifs-utils git tmux curl"

step "2/8 NAS montato in modo permanente ($MNT, utente '$NAS_USER')"
ping -c1 -W2 $NAS_IP >/dev/null && ok "NAS raggiungibile" || { warn "NAS $NAS_IP non risponde: controlla la rete"; exit 1; }
if [ ! -f /root/.nas-cred ] && ! sudo test -f /root/.nas-cred; then
  read -rsp "Password NAS per '$NAS_USER' (non viene mostrata): " NASPW; echo
  printf 'username=%s\npassword=%s\n' "$NAS_USER" "$NASPW" | sudo tee /root/.nas-cred >/dev/null; unset NASPW
  sudo chmod 600 /root/.nas-cred; ok "credenziali in /root/.nas-cred (leggibili solo da root)"
fi
sudo mkdir -p $MNT
LINE="//$NAS_IP/${NAS_SHARE// /\\040} $MNT cifs credentials=/root/.nas-cred,uid=$(id -u),gid=$(id -g),file_mode=0664,dir_mode=0775,vers=3.0,_netdev,nofail,x-systemd.automount,x-systemd.mount-timeout=30 0 0"
grep -q " $MNT cifs" /etc/fstab || { echo "$LINE" | sudo tee -a /etc/fstab >/dev/null; ok "riga aggiunta a /etc/fstab"; }
sudo systemctl daemon-reload; sudo mount $MNT 2>/dev/null || true
ls $MNT/datasets_SINS >/dev/null && ok "NAS montato: $(df -h --output=avail $MNT | tail -1) liberi" || { warn "mount fallito (password giusta? 'sudo mount $MNT' per vedere l'errore)"; exit 1; }
# Gli script del repo usano /mnt/kloster-reichenau: stesso share, quindi basta un link
[ -e /mnt/kloster-reichenau ] || { sudo ln -s $MNT /mnt/kloster-reichenau; ok "/mnt/kloster-reichenau -> $MNT"; }

step "3/8 Codice SINS ($DIR, branch $BRANCH)"
if [ -d $DIR/.git ]; then git -C $DIR pull --ff-only; else git clone -b $BRANCH $REPO $DIR; fi
git -C $DIR submodule update --init --recursive
# Il container vede il NAS in /mnt/nas (file locale, non committato)
cat > $DIR/compose.override.yaml <<EOF
services:
  jo-sim:
    volumes:
      - $MNT:/mnt/nas
EOF
ok "repo pronto; nel container il NAS sara' in /mnt/nas"

step "4/8 Docker + GPU NVIDIA"
command -v docker >/dev/null || sudo apt-get install -y -qq docker.io docker-compose-v2 >/dev/null
sudo usermod -aG docker $USER
if command -v nvidia-smi >/dev/null && nvidia-smi >/dev/null 2>&1; then
  ok "GPU: $(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)"
  if ! dpkg -l nvidia-container-toolkit 2>/dev/null | grep -q ^ii; then
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
    curl -sL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
      | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#' \
      | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
    sudo apt-get update -qq && sudo apt-get install -y -qq nvidia-container-toolkit >/dev/null
    sudo nvidia-ctk runtime configure --runtime=docker >/dev/null && sudo systemctl restart docker
  fi
  ok "nvidia-container-toolkit"
else
  warn "nessuna GPU NVIDIA funzionante: il container jo_sim_sins (runtime: nvidia) non partira'. Installa prima il driver."
fi

step "5/8 Dati di lavoro dal NAS in $DIR/bags (finestre CODa di tuning, Marina di Pisa street)"
D=$MNT/datasets_SINS/derived
mkdir -p $DIR/bags/coda/runs $DIR/bags/mdp/runs $DIR/bags/replay
rsync -a --info=progress2 $D/coda_windows/ $DIR/bags/coda/
rsync -a --info=progress2 --exclude dbg_tracks.csv $D/mdp_street_work/ $DIR/bags/mdp/
# script di replay come sul portatile: run_*.sh + adapter + pesi YOLO (tutti nel repo)
cp $DIR/eval/scripts/*.sh $DIR/eval/adapters/*.py $DIR/bags/replay/ 2>/dev/null
cp $DIR/eval/scripts/run_street.sh $DIR/bags/mdp/
ok "dati in $DIR/bags ($(du -sh $DIR/bags | cut -f1))"

step "6/8 Memoria e note di Claude (dal NAS)"
SLUG=$(echo "$DIR" | sed 's#[/_]#-#g')
mkdir -p ~/.claude/projects/$SLUG/memory ~/Desktop/claude_account_backup
rsync -a $D/claude_handoff/memory/ ~/.claude/projects/$SLUG/memory/
rsync -a $D/claude_handoff/backup/ ~/Desktop/claude_account_backup/
ok "memoria in ~/.claude/projects/$SLUG/memory"

step "7/8 rclone (collegamenti OneDrive)"
command -v rclone >/dev/null || curl -fsSL https://rclone.org/install.sh | sudo bash >/dev/null
[ -f ~/.config/rclone/rclone.conf ] && ok "rclone configurato: $(rclone listremotes | tr '\n' ' ')" \
  || warn "manca ~/.config/rclone/rclone.conf: copialo dal portatile (vedi guida)"

step "8/8 Claude Code + Remote Control sempre attivo"
command -v claude >/dev/null || curl -fsSL https://claude.ai/install.sh | bash
mkdir -p ~/.config/systemd/user
cat > ~/.config/systemd/user/claude-rc.service <<EOF
[Unit]
Description=Claude Code Remote Control (SINS lab) in tmux
After=network-online.target
[Service]
Type=forking
WorkingDirectory=$DIR
ExecStart=/usr/bin/tmux new-session -d -s claude "$HOME/.local/bin/claude remote-control --name 'SINS lab'"
ExecStop=/usr/bin/tmux kill-session -t claude
Restart=on-failure
RestartSec=30
[Install]
WantedBy=default.target
EOF
sudo loginctl enable-linger $USER
systemctl --user daemon-reload
ok "servizio claude-rc creato (si attiva dopo il login a Claude, vedi sotto)"

echo -e "\n\e[1mFATTO. Ora, a mano:\e[0m
  1. Esci e rientra (o riavvia) per usare docker senza sudo.
  2. Login a Claude:            claude      -> /login con l'account claude.ai, poi /exit
  3. Attiva Remote Control:     systemctl --user enable --now claude-rc
  4. Container SINS:            cd $DIR && docker compose up -d --build
  5. Dal portatile/telefono:    claude.ai/code o app Claude -> sessione 'SINS lab'
     (per vederla in diretta sul PC: tmux attach -t claude, poi Ctrl+b d per staccarti)"
