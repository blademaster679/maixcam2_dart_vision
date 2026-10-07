#!/usr/bin/env python3
"""Build benchmark against an already built real MaixCDK application (no image stubs)."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[2]

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--source', type=Path, default=ROOT / 'tools/model/board_video_model_fps.cpp')
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    build = ROOT / 'build'
    flags = {}
    for line in (build / 'main/CMakeFiles/main.dir/flags.make').read_text().splitlines():
        if ' = ' in line:
            key, value = line.split(' = ', 1)
            flags[key] = shlex.split(value)
    link = shlex.split((build / 'CMakeFiles/dart_green_detect.dir/link.txt').read_text())
    source = args.source.resolve()
    obj, exe = out / 'benchmark.o', out / source.stem
    compile_cmd = [link[0], *flags['CXX_DEFINES'], *flags['CXX_INCLUDES'], *flags['CXX_FLAGS'],
                   '-c', str(source), '-o', str(obj)]
    with (out / 'compile.log').open('w') as log:
        subprocess.run(compile_cmd, stdout=log, stderr=subprocess.STDOUT, check=True)
    link[link.index('CMakeFiles/dart_green_detect.dir/exe_src.c.o')] = str(obj)
    link[link.index('-o') + 1] = str(exe)
    with (out / 'link.log').open('w') as log:
        subprocess.run(link, cwd=build, stdout=log, stderr=subprocess.STDOUT, check=True)
    (out / 'build_manifest.json').write_text(json.dumps({
        'compile': compile_cmd, 'link': link, 'link_cwd': str(build),
        'source_sha256': digest(source),
        'decoder_source_sha256': digest(ROOT / 'tools/model/board_video_model_fps.cpp'),
        'nv21_decoder_sha256': digest(ROOT / 'tools/video_replay/board_nv21_replay.cpp'),
        'headers_sha256': {str(p): digest(p) for p in (ROOT / 'main/include/dart').glob('*.hpp')}, 'binary_sha256': digest(exe),
        'application_archive_sha256': digest(build / 'main/libmain.a'),
        'requires': 'same SDK shared libraries as the deployed application',
    }, indent=2) + '\n')
    print(exe)

if __name__ == '__main__':
    main()
