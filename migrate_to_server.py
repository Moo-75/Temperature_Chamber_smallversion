#!/usr/bin/env python3
"""
Run on the Raspberry Pi. Push the CONTENTS of a selected parent folder
(the folder that holds session folders) to the Linux server.

On this network the Pi CAN reach the server (ssh -p 6022) but CANNOT reach the
lab PC, so we push to the server instead of the PC.

Only "parent" folders are offered for selection: a parent is a folder whose
immediate subfolders are experiment session folders (folders that directly
contain files like Temperature_*.csv / Video_*.mp4 / SensorTime_*.csv /
TD_*_trial-wise.csv). Individual session folders and unrelated folders are NOT
listed.

The parent folder itself is NOT copied; only its contents are. Example:

    Pi:      ~/Desktop/BRL/{session1, session2}
    pick "BRL"  ->
    server:  /data/Siheon_chamber_data/{session1, session2}

Usage:
    python3 migrate_to_server.py
    python3 migrate_to_server.py --target myid@10.140.5.118 --port 6022
"""

from __future__ import annotations

import argparse
import fnmatch
import os
import re
import shlex
import shutil
import subprocess
import sys
from pathlib import Path


# Server login. 'user' is the actual account name on 10.140.5.118.
# Override with --target or the CHAMBER_SERVER_TARGET environment variable.
DEFAULT_TARGET = "user@10.140.5.118"
DEFAULT_PORT = 6022
DEFAULT_DEST = "/data/Siheon_chamber_data"

EXPERIMENT_PATTERNS = (
    "Temperature_*.csv",
    "SensorTime_*.csv",
    "Video_*.mp4",
    "TD_*_trial-wise.csv",
    "TD_*.csv",
    "TD_*",
    "temp_test_*.csv",
)

# Set in main() when sshpass-based passwordless auth is enabled.
_EXTRA_ENV: dict = {}
_SSHPASS_PREFIX: list = []


def shell_quote(value) -> str:
    return shlex.quote(str(value))


def matches_experiment(name: str) -> bool:
    return any(fnmatch.fnmatch(name, pattern) for pattern in EXPERIMENT_PATTERNS)


def is_session_dir(path: Path) -> bool:
    """A session folder directly contains at least one experiment file."""
    try:
        with os.scandir(path) as entries:
            for entry in entries:
                if entry.is_file() and matches_experiment(entry.name):
                    return True
    except OSError:
        pass
    return False


def summarize(path: Path) -> tuple[int, int]:
    file_count = 0
    byte_count = 0
    for root, dirnames, filenames in os.walk(path):
        dirnames[:] = [name for name in dirnames if not name.startswith(".")]
        for name in filenames:
            try:
                byte_count += (Path(root) / name).stat().st_size
                file_count += 1
            except OSError:
                continue
    return file_count, byte_count


def format_bytes(size: int) -> str:
    value = float(size)
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if value < 1024 or unit == "TB":
            return f"{int(value)} B" if unit == "B" else f"{value:.1f} {unit}"
        value /= 1024
    return f"{size} B"


def default_bases() -> list[Path]:
    home = Path.home()
    cwd = Path.cwd()
    candidates = [home / "Desktop", cwd.parent, cwd, home]
    seen: set[Path] = set()
    bases: list[Path] = []
    for candidate in candidates:
        try:
            resolved = candidate.expanduser().resolve()
        except OSError:
            continue
        if resolved in seen or not resolved.is_dir():
            continue
        seen.add(resolved)
        bases.append(resolved)
    return bases


def discover_parents(bases: list[Path], max_depth: int) -> list[dict]:
    """Find folders whose immediate subfolders are experiment session folders."""
    parents: dict[Path, dict] = {}
    for base in bases:
        base = base.resolve()
        if not base.is_dir():
            continue
        for root, dirnames, filenames in os.walk(base):
            root_path = Path(root)
            visible_dirs = [name for name in dirnames if not name.startswith(".")]
            dirnames[:] = visible_dirs

            depth = len(root_path.relative_to(base).parts)
            # Decide (using the pre-prune list) whether this folder is a parent,
            # then stop descending past max_depth.
            session_children = [
                name for name in visible_dirs if is_session_dir(root_path / name)
            ]
            if depth >= max_depth:
                dirnames[:] = []

            if not session_children:
                continue
            try:
                key = root_path.resolve()
            except OSError:
                continue
            # Do not allow root, home, or Desktop itself to be treated as a parent folder
            # (which would transfer and attempt to delete the entire Desktop/Home).
            home_resolved = Path.home().resolve()
            if key in {Path("/"), home_resolved, home_resolved / "Desktop"}:
                continue
            if key in parents:
                continue
            files, size = summarize(root_path)
            parents[key] = {
                "path": root_path,
                "sessions": len(session_children),
                "files": files,
                "size": size,
            }
    return sorted(parents.values(), key=lambda item: str(item["path"]).lower())


def _control_opts() -> list[str]:
    # Reuse one authenticated master connection for every ssh/scp/rsync in this
    # run, so the password is asked at most once instead of once per command.
    return [
        "-o", "ControlMaster=auto",
        "-o", "ControlPath=/tmp/migrate_ctl_%r@%h:%p",
        "-o", "ControlPersist=120",
    ]


def _popen_env():
    if _EXTRA_ENV:
        env = dict(os.environ)
        env.update(_EXTRA_ENV)
        return env
    return None


def _wrap(cmd: list[str]) -> list[str]:
    # Prepend sshpass (if enabled) so the whole ssh/scp/rsync command is fed the
    # password non-interactively.
    return [*_SSHPASS_PREFIX, *cmd]


def ssh_base(args: argparse.Namespace) -> list[str]:
    cmd = ["ssh", "-p", str(args.port), *_control_opts()]
    if args.identity_file:
        cmd += ["-i", args.identity_file]
    for option in args.ssh_option:
        cmd += ["-o", option]
    return cmd


def ssh_command_string(args: argparse.Namespace) -> str:
    parts = ["ssh", "-p", str(args.port), *_control_opts()]
    if args.identity_file:
        parts += ["-i", args.identity_file]
    for option in args.ssh_option:
        parts += ["-o", option]
    return " ".join(shell_quote(part) for part in parts)


def run_printed(cmd: list[str]) -> None:
    print("+ " + " ".join(shell_quote(part) for part in cmd))
    subprocess.run(cmd, check=True, env=_popen_env())


def remote_has_rsync(args: argparse.Namespace) -> bool:
    cmd = _wrap([*ssh_base(args), args.target, "command -v rsync >/dev/null 2>&1"])
    return subprocess.run(cmd, env=_popen_env()).returncode == 0


def ensure_remote_dir(args: argparse.Namespace) -> None:
    run_printed(_wrap([*ssh_base(args), args.target, f"mkdir -p {shell_quote(args.dest)}"]))


# Matches the fields of an `rsync --info=progress2` line, e.g.
#   722.08M  19%    8.88MB/s    0:05:33 (xfr#14, to-chk=18/39)
_PROGRESS_RE = re.compile(r"(\d+)%\s+(\S+/s)\s+(\d+:\d\d:\d\d)")


def run_rsync_with_bar(cmd: list[str]) -> None:
    print("+ " + " ".join(shell_quote(part) for part in cmd))
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=_popen_env(),
    )
    width = 30
    buffer = ""
    bar_on_screen = False
    try:
        while True:
            char = proc.stdout.read(1)
            if not char:
                break
            if char in ("\r", "\n"):
                line = buffer.strip()
                buffer = ""
                match = _PROGRESS_RE.search(line)
                if match:
                    pct = int(match.group(1))
                    speed = match.group(2)
                    eta = match.group(3)
                    filled = int(width * pct / 100)
                    bar = "#" * filled + "-" * (width - filled)
                    sys.stdout.write(f"\rCopying: [{bar}] {pct:3d}%  {speed:>10}  ETA {eta}    ")
                    sys.stdout.flush()
                    bar_on_screen = True
                elif line:
                    if bar_on_screen:
                        sys.stdout.write("\n")
                        bar_on_screen = False
                    print(line)
            else:
                buffer += char
    finally:
        proc.wait()
    if bar_on_screen:
        sys.stdout.write("\n")
        sys.stdout.flush()
    if proc.returncode != 0:
        raise subprocess.CalledProcessError(proc.returncode, cmd)


def transfer(args: argparse.Namespace, parent: Path) -> None:
    ensure_remote_dir(args)
    dest = args.dest.rstrip("/")
    remote_dest = f"{args.target}:{shell_quote(dest + '/')}"

    if shutil.which("rsync") and remote_has_rsync(args):
        cmd = _wrap([
            "rsync",
            "-ah",
            "--info=progress2",
            "--partial",
            "-e",
            ssh_command_string(args),
            str(parent) + os.sep,   # trailing slash => copy the CONTENTS, not the folder
            remote_dest,
        ])
        run_rsync_with_bar(cmd)
        return

    # scp fallback: copy each immediate child into the destination directory.
    print("rsync unavailable on one side; falling back to scp per item.")
    scp = ["scp", "-P", str(args.port), *_control_opts()]
    if args.identity_file:
        scp += ["-i", args.identity_file]
    for option in args.ssh_option:
        scp += ["-o", option]
    for child in sorted(parent.iterdir(), key=lambda p: p.name):
        if child.name.startswith("."):
            continue
        run_printed(_wrap([*scp, "-r", str(child), remote_dest]))


def prompt_choice(parents: list[dict]) -> Path:
    print("\nParent folders on the Raspberry Pi (each holds session folders):")
    for index, item in enumerate(parents, start=1):
        print(
            f"  {index:>2}. {item['path']}  "
            f"({item['sessions']} sessions, {item['files']} files, {format_bytes(item['size'])})"
        )
    while True:
        raw = input("\nSend which parent folder's contents to the server? number, or q to quit: ").strip()
        if raw.lower() in {"q", "quit", "exit"}:
            raise SystemExit(0)
        try:
            index = int(raw)
        except ValueError:
            print("Please enter a number from the list.")
            continue
        if 1 <= index <= len(parents):
            return parents[index - 1]["path"]
        print("That number is not in the list.")


def maybe_delete(parent: Path) -> None:
    raw = input(f"\nDelete the copied contents from the Pi folder {parent}? [y/N]: ").strip().lower()
    if raw not in {"y", "yes"}:
        print("Left the files in place on the Pi.")
        return

    resolved = parent.resolve()
    home = Path.home().resolve()
    if resolved in {Path("/"), home, home / "Desktop"} or len(resolved.parts) < 4:
        raise SystemExit(f"Refusing to delete unsafe path: {resolved}")

    for child in parent.iterdir():
        if child.is_dir() and not child.is_symlink():
            shutil.rmtree(child)
        else:
            child.unlink()
    print("Deleted the folder contents on the Pi (kept the parent folder itself).")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="On the Pi: push a parent folder's contents to the Linux server.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--target",
        default=os.environ.get("CHAMBER_SERVER_TARGET", DEFAULT_TARGET),
        help="Server SSH target as user@host. Replace 'user' with your real login.",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("CHAMBER_SERVER_PORT", DEFAULT_PORT)),
        help="Server SSH port.",
    )
    parser.add_argument(
        "--dest",
        default=os.environ.get("CHAMBER_SERVER_DEST", DEFAULT_DEST),
        help="Destination directory on the server. Session folders are placed directly inside it.",
    )
    parser.add_argument(
        "--search-base",
        action="append",
        help="Local directory on the Pi to scan. Repeat for several. Default: ~/Desktop, project parent, cwd, ~.",
    )
    parser.add_argument("--max-depth", type=int, default=4, help="How deep to scan below each base.")
    parser.add_argument("--identity-file", default=None, help="SSH private key path.")
    parser.add_argument(
        "--password-env",
        default="CHAMBER_SERVER_PASSWORD",
        help="Env var holding the server password. If set and 'sshpass' is installed, runs with no prompts.",
    )
    parser.add_argument("--ssh-option", action="append", default=[], help="Extra ssh -o option. Repeat for multiple.")
    parser.add_argument("--no-delete-prompt", action="store_true", help="Skip the delete-after-copy step.")
    args = parser.parse_args(argv)
    if args.max_depth < 1:
        parser.error("--max-depth must be at least 1")
    return args


def main(argv: list[str] | None = None) -> int:
    global _SSHPASS_PREFIX, _EXTRA_ENV
    args = parse_args(sys.argv[1:] if argv is None else argv)

    password = os.environ.get(args.password_env) if args.password_env else None
    if password:
        if shutil.which("sshpass"):
            _SSHPASS_PREFIX = ["sshpass", "-e"]
            _EXTRA_ENV = {"SSHPASS": password}
            print(f"[Auth] Using sshpass with the password from ${args.password_env}; no prompts.\n")
        else:
            print(f"[Auth] ${args.password_env} is set but 'sshpass' is not installed.")
            print("       Install it (sudo apt install sshpass) or set up SSH keys for passwordless login.\n")

    bases = [Path(item).expanduser() for item in args.search_base] if args.search_base else default_bases()
    parents = discover_parents(bases, args.max_depth)
    if not parents:
        print("No parent folders containing session folders were found.")
        print("A session folder directly contains files like Temperature_*.csv or Video_*.mp4.")
        print("Try --search-base <dir> or a larger --max-depth.")
        return 1

    parent = prompt_choice(parents)
    print(f"\nSending CONTENTS of {parent}")
    print(f"     -> {args.target}:{args.dest}  (port {args.port})")
    try:
        transfer(args, parent)
        print("Transfer complete.")
    except subprocess.CalledProcessError as e:
        print(f"\n[오류] 서버 전송 중 오류가 발생했습니다 (종료 코드 {e.returncode}).")
        print("확인 사항:")
        print(f"  1. SSH 대상 계정 및 IP: {args.target}")
        print(f"  2. SSH 포트: {args.port}")
        print(f"  3. 서버 저장 경로: {args.dest}")
        print("  4. 서버 연결 상태 (VPN/내부망) 및 비밀번호 확인")
        print(f"  예시: python3 migrate_to_server.py --target your_id@10.140.5.118 --port {args.port}")
        return 1

    if not args.no_delete_prompt:
        maybe_delete(parent)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
