#!/usr/bin/env bash
# Run an experiment inside tmux so it keeps going when the remote shell
# (Raspberry Pi Connect, SSH) drops or the connecting computer shuts down.
#
#   bash run_detached.sh                      # python3 maintemp.py
#   bash run_detached.sh python3 perf_bands.py --plan pilot
#   bash run_detached.sh                      # again later: re-attach to the running session
#
# Answer the prompts as usual. Detach on purpose with Ctrl-b then d.
set -euo pipefail

SESSION="${SESSION:-chamber}"
cd "$(dirname "$(readlink -f "$0")")"

if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux가 없습니다. 먼저: sudo apt install -y tmux"
    exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "이미 '$SESSION' 세션이 있습니다. 다시 접속합니다 (분리: Ctrl-b 다음 d)."
    exec tmux attach -t "$SESSION"
fi

CMD=("$@")
if [ ${#CMD[@]} -eq 0 ]; then
    CMD=(python3 maintemp.py)
fi
printf -v CMD_STR '%q ' "${CMD[@]}"

echo "tmux 세션 '$SESSION' 에서 실행: ${CMD[*]}"
echo "연결이 끊겨도 계속 돕니다. 다시 붙으려면: bash run_detached.sh"
exec tmux new-session -s "$SESSION" "${CMD_STR}; echo; echo '[끝] Enter를 누르면 이 창이 닫힙니다.'; read -r _"
