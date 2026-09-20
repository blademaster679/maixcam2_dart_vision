#!/usr/bin/env python3
"""Build isolated AArch64 RGB or shared NV21-pipeline replay with cached SDK libs."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk", type=Path, default=Path.home() / "maix/MaixCDK")
    parser.add_argument("--out", type=Path, default=ROOT / ".maixpy/clip13-board-replay")
    parser.add_argument("--pipeline", choices=("rgb", "nv21", "offline"), default="rgb")
    args = parser.parse_args()
    sdk, out = args.sdk.resolve(), args.out.resolve()
    if out.exists():
        raise SystemExit("choose a new output directory to preserve previous benchmark builds")
    compiler = next((sdk / "dl/extracted/toolchains/maixcam2").glob("*/bin/aarch64-none-linux-gnu-g++"))
    opencv = next((sdk / "dl/extracted/opencv/opencv4").glob("opencv4_lib_maixcam2_glibc_*"))
    ffmpeg = next((sdk / "dl/extracted/ffmpeg_srcs").glob("ffmpeg_maixcam2_libs_*"))
    executable = {"rgb": "board_replay_benchmark", "nv21": "board_nv21_replay",
                  "offline": "board_nv21_offline"}[args.pipeline]
    sources = [HERE / (executable + ".cpp"), HERE / "maix_image_opencv.cpp"]
    sources += [ROOT / "main/src" / name for name in (
        "config.cpp", "green_detector.cpp", "green_detector_core.cpp",
        "target_fusion.cpp", "target_json.cpp", "visual_motion.cpp")]
    if args.pipeline in ("nv21", "offline"):
        sources.append(ROOT / "main/src/nv21_pipeline.cpp")
    inputs = sources + list((ROOT / "main/include/dart").glob("*.hpp")) + [HERE / "maix_image.hpp"]
    if args.pipeline == "offline":
        # The isolated sequential runner reuses this translation unit's decoder.
        inputs.append(HERE / "board_nv21_replay.cpp")
    hashes = {str(path): sha256(path) for path in inputs}
    out.mkdir(parents=True)
    (out / "BUILD_INCOMPLETE").write_text("Build in progress; do not deploy.\n")
    common = [str(compiler), "-std=c++17", "-O2", "-DNDEBUG", "-Wall", "-Wextra", "-Wpedantic",
              "-pthread", "-I" + str(HERE), "-I" + str(ROOT / "main/include"),
              "-I" + str(opencv / "include/opencv4"), "-I" + str(ffmpeg / "include")]
    commands, objects = [], []
    for source in sources:
        obj = out / (source.stem + ".o")
        command = common + ["-c", str(source), "-o", str(obj)]
        with (out / (source.stem + ".log")).open("w") as log:
            subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT)
        commands.append(command)
        objects.append(str(obj))
    command = [str(compiler), "-pthread", *objects, "-Wl,--as-needed",
               "-Wl,-rpath,$ORIGIN/dl_lib", "-Wl,--disable-new-dtags",
               "-L" + str(opencv / "dl_lib"), "-L" + str(ffmpeg / "lib"),
               "-Wl,-rpath-link," + str(opencv / "dl_lib"), "-Wl,-rpath-link," + str(ffmpeg / "lib"),
               "-lopencv_imgproc", "-lopencv_core", "-lavformat", "-lavcodec", "-lavutil", "-lswscale",
               "-o", str(out / executable)]
    with (out / "link.log").open("w") as log:
        subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT)
    commands.append(command)
    library_dir = out / "dl_lib"
    library_dir.mkdir()
    libraries = []
    for parent, names in [(ffmpeg / "lib", ["avformat", "avcodec", "avutil", "swscale", "swresample"]),
                          (opencv / "dl_lib", ["opencv_core", "opencv_imgproc"])]:
        for name in names:
            path = (parent / ("lib" + name + ".so")).resolve()
            details = subprocess.check_output(["readelf", "-d", str(path)], text=True)
            soname = re.search(r"\(SONAME\).*\[(.*?)\]", details).group(1)
            target = library_dir / soname
            shutil.copy2(path, target)
            libraries.append({"file": str(target.relative_to(out)), "sha256": sha256(target)})
    if any(sha256(Path(path)) != digest for path, digest in hashes.items()):
        raise SystemExit("source changed during build; BUILD_INCOMPLETE retained")
    manifest = {"benchmark": {"rgb": "rgb_software_decode_replay", "nv21": "nv21_cached_pipeline_replay",
                              "offline": "offline_continuous_nv21_roi"}[args.pipeline], "compiler": str(compiler),
                "commands": commands, "source_sha256": hashes, "libraries": libraries,
                "executable": executable, "executable_sha256": sha256(out / executable),
                "scope": "isolated executable and local runtime libs; no live camera or system replacement"}
    (out / "build_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (out / "BUILD_INCOMPLETE").unlink()
    print(out)


if __name__ == "__main__":
    main()
