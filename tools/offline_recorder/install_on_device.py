#!/usr/bin/env python3
"""Install the offline recorder through the board's app_store."""
from __future__ import annotations

import argparse
import hashlib
import re
import subprocess
import sys
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("package", type=Path,
                        nargs="?", default=Path("dist/dart_data_recorder_v0.4.3.zip"))
    parser.add_argument("--skill", type=Path,
                        default=Path("/home/blade_master/pnx/maixpy-skill/maixpy"))
    args = parser.parse_args()
    package = args.package.resolve()
    if not package.is_file() or not re.fullmatch(r"[A-Za-z0-9_.-]+\.zip", package.name):
        raise SystemExit(f"invalid package: {package}")

    sys.path.insert(0, str(args.skill.resolve() / "scripts"))
    from maixpy_skill.config import get_device
    from maixpy_skill.ssh import command_env, run_ssh, scp_to_command

    device = get_device()
    remote = f"/root/{package.name}"
    expected = hashlib.sha256(package.read_bytes()).hexdigest()
    transfer = subprocess.run(scp_to_command(device, str(package), remote),
                              env=command_env(device), check=False)
    if transfer.returncode:
        return transfer.returncode

    command = (
        f"test \"$(sha256sum {remote} | awk '{{print $1}}')\" = {expected} && "
        f"/maixapp/apps/app_store/app_store install {remote}; status=$?; "
        f"rm -f {remote}; "
        "test $status -eq 0 || exit $status; "
        "test -f /maixapp/apps/dart_data_recorder/app.yaml || exit 20; "
        "test -f /maixapp/apps/dart_data_recorder/main.sh || exit 20; "
        "test -f /maixapp/apps/dart_data_recorder/confirm_start.py || exit 20; "
        "test -f /maixapp/apps/dart_data_recorder/direct_record || exit 20; "
        "test -f /maixapp/apps/dart_data_recorder/libsns_os04a10.so || exit 20; "
        "chmod +x /maixapp/apps/dart_data_recorder/direct_record || exit 20; "
        "dependencies=$(LD_LIBRARY_PATH=/maixapp/apps/dart_data_recorder:/opt/lib:/usr/lib:/lib "
        "ldd /maixapp/apps/dart_data_recorder/direct_record 2>&1); "
        "printf '%s\\n' \"$dependencies\"; "
        "case \"$dependencies\" in *'not found'*) exit 21;; esac; "
        "sha256sum /maixapp/apps/dart_data_recorder/direct_record "
        "/maixapp/apps/dart_data_recorder/libsns_os04a10.so; "
        "echo recorder_install=ok"
    )
    result = run_ssh(device, command, timeout=90)
    print(result.stdout, end="")
    if result.stderr:
        print(result.stderr, file=sys.stderr, end="")
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
