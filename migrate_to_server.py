#!/usr/bin/env python3
"""
Run on the Raspberry Pi. Push the contents of experiment parent folders or
standalone session folders to the Linux server.

On this network the Pi CAN reach the server (ssh -p 6022) but CANNOT reach the
lab PC, so we push to the server instead of the PC.

Supports two folder structures:
1. Parent folder (e.g. ~/Desktop/OBT, ~/Desktop/TEMP_TEST):
   A folder containing multiple session subfolders. The session folders inside it
   are transferred to /data/Siheon_chamber_data/<session>.
2. Standalone Session folder (e.g. ~/Desktop/test):
   A folder directly containing experiment files (Temperature_*.csv, etc.).
   The folder itself is transferred to /data/Siheon_chamber_data/<session>.

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


def discover_targets(bases: list[Path], max_depth: int) -> list[dict]:
    """
    Find:
    1) Parent folders (folders whose immediate subfolders are session folders)
    2) Standalone session folders (folders directly containing experiment files)
    """
    parents: dict[Path, dict] = {}
    standalone_sessions: dict[Path, dict] = {}

    home_resolved = Path.home().resolve()
    protected_roots = {Path("/"), home_resolved, home_resolved / "Desktop"}

    for base in bases:
        base = base.resolve()
        if not base.is_dir():
            continue
        for root, dirnames, filenames in os.walk(base):
            root_path = Path(root)
            visible_dirs = [name for name in dirnames if not name.startswith(".")]
            dirnames[:] = visible_dirs

            depth = len(root_path.relative_to(base).parts)
            session_children = [
                name for name in visible_dirs if is_session_dir(root_path / name)
            ]
            if depth >= max_depth:
                dirnames[:] = []

            try:
                key = root_path.resolve()
            except OSError:
                continue

            # Standalone session folder
            if is_session_dir(root_path) and key not in protected_roots:
                files, size = summarize(root_path)
                standalone_sessions[key] = {
                    "path": root_path,
                    "kind": "Session",
                    "sessions": 1,
                    "files": files,
                    "size": size,
                }

            if not session_children:
                continue
            if key in protected_roots:
                continue
            if key in parents:
                continue

            files, size = summarize(root_path)
            parents[key] = {
                "path": root_path,
                "kind": "Parent",
                "sessions": len(session_children),
                "files": files,
                "size": size,
            }

    targets: list[dict] = []
    for parent_info in parents.values():
        targets.append(parent_info)

    parent_paths = [p["path"] for p in parents.values()]
    for session_info in standalone_sessions.values():
        session_path = session_info["path"]
        is_inside_parent = False
        for p_path in parent_paths:
            try:
                session_path.relative_to(p_path)
                is_inside_parent = True
                break
            except ValueError:
                pass
        if not is_inside_parent:
            targets.append(session_info)

    return sorted(targets, key=lambda item: str(item["path"]).lower())


def _control_opts() -> list[str]:
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


def transfer(args: argparse.Namespace, target_info: dict) -> None:
    ensure_remote_dir(args)
    dest = args.dest.rstrip("/")
    remote_dest = f"{args.target}:{shell_quote(dest + '/')}"
    target_path = target_info["path"]
    is_parent = (target_info["kind"] == "Parent")

    if shutil.which("rsync") and remote_has_rsync(args):
        # Trailing slash for parent => copy the CONTENTS
        # No trailing slash for session => copy the FOLDER itself
        src = str(target_path) + (os.sep if is_parent else "")
        cmd = _wrap([
            "rsync",
            "-ah",
            "--info=progress2",
            "--partial",
            "-e",
            ssh_command_string(args),
            src,
            remote_dest,
        ])
        run_rsync_with_bar(cmd)
        return

    # scp fallback
    print("rsync unavailable on one side; falling back to scp per item.")
    scp = ["scp", "-P", str(args.port), *_control_opts()]
    if args.identity_file:
        scp += ["-i", args.identity_file]
    for option in args.ssh_option:
        scp += ["-o", option]

    if is_parent:
        for child in sorted(target_path.iterdir(), key=lambda p: p.name):
            if child.name.startswith("."):
                continue
            run_printed(_wrap([*scp, "-r", str(child), remote_dest]))
    else:
        run_printed(_wrap([*scp, "-r", str(target_path), remote_dest]))


def prompt_choice(targets: list[dict]) -> dict:
    print("\nAvailable folders to transfer to the Linux server:")
    for index, item in enumerate(targets, start=1):
        kind_tag = f"[{item['kind']}]"
        print(
            f"  {index:>2}. {kind_tag:<9} {item['path']}  "
            f"({item['sessions']} sessions, {item['files']} files, {format_bytes(item['size'])})"
        )
    while True:
        raw = input("\nSend which folder to the server? number, or q to quit: ").strip()
        if raw.lower() in {"q", "quit", "exit"}:
            raise SystemExit(0)
        try:
            index = int(raw)
        except ValueError:
            print("Please enter a number from the list.")
            continue
        if 1 <= index <= len(targets):
            return targets[index - 1]
        print("That number is not in the list.")


def maybe_delete(target_info: dict) -> None:
    target_path = target_info["path"]
    is_parent = (target_info["kind"] == "Parent")
    action_str = f"the copied contents from the folder {target_path}" if is_parent else f"the session folder {target_path}"
    raw = input(f"\nDelete {action_str}? [y/N]: ").strip().lower()
    if raw not in {"y", "yes"}:
        print("Left the files in place on the Pi.")
        return

    resolved = target_path.resolve()
    home = Path.home().resolve()
    if resolved in {Path("/"), home, home / "Desktop"} or len(resolved.parts) < 3:
        raise SystemExit(f"Refusing to delete unsafe path: {resolved}")

    if is_parent:
        for child in target_path.iterdir():
            if child.is_dir() and not child.is_symlink():
                shutil.rmtree(child)
            else:
                child.unlink()
        print(f"Deleted the folder contents on the Pi (kept {target_path} itself).")
    else:
        shutil.rmtree(target_path)
        print(f"Deleted session folder {target_path} on the Pi.")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="On the Pi: push experiment data to the Linux server.",
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
    targets = discover_targets(bases, args.max_depth)
    if not targets:
        print("No parent folders or session folders containing experiment data were found.")
        print("Expected files: Temperature_*.csv, SensorTime_*.csv, TD_*.csv, temp_test_*.csv, etc.")
        print("Try --search-base <dir> or a larger --max-depth.")
        return 1

    target_info = prompt_choice(targets)
    target_path = target_info["path"]
    if target_info["kind"] == "Parent":
        print(f"\nSending CONTENTS of {target_path}")
    else:
        print(f"\nSending FOLDER {target_path}")
    print(f"     -> {args.target}:{args.dest}  (port {args.port})")

    try:
        transfer(args, target_info)
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
        maybe_delete(target_info)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
