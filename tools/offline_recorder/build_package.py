#!/usr/bin/env python3
"""Build a standalone MaixAPP package around the verified direct VIN recorder."""
from __future__ import annotations

import argparse
import hashlib
import json
import stat
import zipfile
from pathlib import Path


HERE = Path(__file__).resolve().parent
APP_ID = "dart_data_recorder"
VERSION = "0.4.3"


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--driver-build", type=Path, required=True)
    parser.add_argument("--sdk", type=Path, default=Path.home() / "maix/MaixCDK")
    parser.add_argument("--output", type=Path,
                        default=Path(f"dist/{APP_ID}_v{VERSION}.zip"))
    args = parser.parse_args()

    build = args.driver_build.resolve()
    sdk = args.sdk.resolve()
    if (build.joinpath("BUILD_INCOMPLETE").exists()):
        raise SystemExit("driver build is incomplete")
    manifest_path = build / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("profile", {}).get("status") != "official_source_test":
        raise SystemExit("an official OS04A10 build is required")

    sources = {
        "app.yaml": HERE / "app.yaml",
        "main.sh": HERE / "main.sh",
        "recorderctl.py": HERE / "recorderctl.py",
        "recorder.conf": HERE / "recorder.conf",
        "confirm_start.py": HERE / "confirm_start.py",
        "show_status.py": HERE / "show_status.py",
        "direct_record": build / "direct_record",
        "libsns_os04a10.so": build / "libsns_os04a10.so",
        "libms_asr_ax630c.so": sdk / "components/nn/lib/libms_asr_ax630c.so",
        "libasound.so.2": sdk / "components/3rd_party/alsa_lib/lib/maixcam2/libasound.so",
        "libatopology.so.2": sdk / "components/3rd_party/alsa_lib/lib/maixcam2/libatopology.so",
        "libdatachannel.so": sdk / "components/3rd_party/datachannel/lib/maixcam2/libdatachannel.so",
        "libonnxruntime.so.1": sdk / "dl/extracted/onnxruntime_srcs/maixcam2_onnxruntime_v1.22.0/lib/libonnxruntime.so.1.22.0",
        "libtinyalsa.so.2": sdk / "dl/extracted/maixcam2_msp_srcs/maixcam2_msp_arm64_glibc_v3.0.0_20250319114413/out/arm64_glibc/third-party/lib/libtinyalsa.so.2.0.0",
    }
    for name, path in sources.items():
        if not path.is_file() or path.stat().st_size == 0:
            raise SystemExit(f"missing package input: {name}: {path}")

    build_manifest = {
        "schema_version": 1,
        "app_id": APP_ID,
        "version": VERSION,
        "driver_build": build.name,
        "driver_profile": manifest.get("profile"),
        "files": {name: sha256(path.read_bytes()) for name, path in sources.items()},
        "scope": "Standalone full180 recorder; no system library replacement or autostart mutation",
    }
    sources["build_manifest.json"] = None

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(args.output, "w", compression=zipfile.ZIP_DEFLATED,
                         compresslevel=9) as archive:
        for name, path in sources.items():
            data = (json.dumps(build_manifest, indent=2, sort_keys=True) + "\n").encode() \
                if path is None else path.read_bytes()
            info = zipfile.ZipInfo(f"{APP_ID}/{name}")
            info.external_attr = ((stat.S_IFREG | (0o755 if name in {
                "main.sh", "confirm_start.py", "show_status.py", "direct_record"
            } else 0o644)) << 16)
            info.compress_type = zipfile.ZIP_DEFLATED
            archive.writestr(info, data)

    print(args.output.resolve())
    print(f"sha256={sha256(args.output.read_bytes())}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
