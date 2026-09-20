#!/usr/bin/env python3
"""Run an isolated C++ RAW benchmark through the supplied MaixPy SSH helpers.

No persistent device files are replaced. Each run gets a /tmp directory. The
launcher is restored if it was active; refuse to displace other running apps.
"""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys
import tarfile
import uuid


def development_enter_command(launcher, headless=False, pause_camera=False):
    """Select launcher isolation without initializing a display in headless mode.

    This avoids an unnecessary display transaction; it cannot clear a stuck
    kernel I2C transfer or make an unavailable camera usable.
    """
    if not headless:
        return launcher.enter_command()
    permitted_camera = ' && $i != "/maixapp/apps/camera/camera"' if pause_camera else ''
    unrelated_apps = (
        "ps -eo args | awk '{for (i=1; i<=NF; ++i) if "
        "($i ~ /^\\/maixapp\\/apps\\// "
        "&& $i !~ /^\\/maixapp\\/apps\\/launcher\\//"
        + permitted_camera + ") {print; break}}'"
    )
    # Stop respawning first, then recheck after staging before the helper may
    # terminate any apps, including interpreter-launched app scripts. Only the
    # explicitly authorized camera may remain.
    return (
        f"{launcher.kill_launcher_daemon_command()}; "
        f"unrelated_apps=$({unrelated_apps}); "
        'if [ -n "$unrelated_apps" ]; then '
        'printf \'Another app is active: %s\\n\' "$unrelated_apps" >&2; exit 1; fi; '
        f"{launcher.ensure_maixapp_apps_stopped_command()}; "
        "echo development_mode=entered_headless"
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--skill', type=Path, default=Path('/home/blade_master/pnx/maixpy-skill/maixpy'))
    ap.add_argument('--driver-build', type=Path, default=Path('.maixpy/os04a10-build'))
    ap.add_argument('--sdk', type=Path, default=Path.home()/'maix/MaixCDK')
    ap.add_argument('--seconds', type=int, default=10)
    ap.add_argument('--business-config', type=Path)
    ap.add_argument('--business-idle', action='store_true')
    ap.add_argument('--business-stress', action='store_true')
    ap.add_argument('--official-mode', choices=['full60','full120','full180','crop240','crop360'])
    ap.add_argument('--camera-api', action='store_true', help='Use the isolated official Camera API probe')
    ap.add_argument('--direct-venc', action='store_true', help='Record VIN NV12 with asynchronous AX VENC, bypassing Camera/IVPS copies')
    ap.add_argument('--record', action='store_true', help='Record with the Camera API probe, maximum 30 seconds')
    ap.add_argument('--burst', action='store_true', help='Save one second of packed 640x360 NV21 in bounded RAM, then write after capture')
    ap.add_argument('--realtime', action='store_true', help='Use SCHED_FIFO priority 10 for the isolated direct acquisition thread')
    ap.add_argument('--max-mipi-errors', type=int, default=0,
                    help='Direct-VENC only: retain up to this many recovered receiver errors')
    ap.add_argument('--venc-retry', action='store_true', help='Retry VENC queue-full on the same retained frame, bounded to 20ms')
    ap.add_argument('--venc-worker', action='store_true', help='Transfer VIN frame ownership to a bounded 32-image encoding worker')
    ap.add_argument('--itp-depth', type=int, choices=[1,4,8], help='Configure and read back the VIN ITP source queue depth')
    ap.add_argument('--venc-copy', action='store_true', help='Diagnostic: copy NV12 pixels to a user pool before encoding')
    ap.add_argument('--venc-fps', type=int, choices=[180,360], help='Diagnostic encoder RC rate; sensor rate is unchanged')
    ap.add_argument('--venc-depth', type=int, choices=[4,8], help='Diagnostic VENC input/output FIFO depth, read back from SDK')
    ap.add_argument('--exercise-switch', action='store_true', help='Exercise Camera FPS/ROI/reopen APIs before measurement')
    ap.add_argument('--no-health', action='store_true', help='Camera diagnostic: omit in-loop health reads; retain before/after readback')
    ap.add_argument('--observe-gaps', action='store_true', help='Collect the full capture diagnostic window despite recovered sequence/MIPI faults; retain a nonzero result')
    ap.add_argument('--gdb', action='store_true', help='Capture a diagnostic backtrace using the device debugger')
    ap.add_argument('--system-media-lib', action='store_true', help='Use the media library installed with the device firmware')
    ap.add_argument('--remote-root', choices=['/tmp','/root/os04a10-tests'], default='/tmp', help='Use device storage for large recordings/long-run metadata')
    ap.add_argument('--experimental', action='store_true')
    ap.add_argument('--hfr', action='store_true')
    ap.add_argument('--sample-raw', action='store_true', help='Save one diagnostic RAW frame during warmup')
    ap.add_argument('--crop-probe', action='store_true', help='Select the explicitly inferred crop candidate')
    ap.add_argument('--fullfov-probe', action='store_true', help='Select native full-array timing control (not 360fps)')
    ap.add_argument('--pause-camera-app', action='store_true', help='Temporarily pause the official camera app and ask the launcher to restore it')
    ap.add_argument('--headless', action='store_true', help='Stop launcher/apps without initializing or refreshing a display; preserves launcher restoration')
    ap.add_argument('--nv21', action='store_true', help='Measure uncompressed VIN channel 0 NV21 instead of IFE RAW')
    ap.add_argument('--sample-frame', action='store_true', help='Save one diagnostic frame during warmup')
    ap.add_argument('--queue-depth', type=int, choices=range(4,33), help='Default: RAW 16, NV21 4')
    args = ap.parse_args()
    if args.business_config and (args.official_mode!='full180' or not args.nv21 or args.camera_api or args.record or args.burst):
        ap.error('Business adapter requires full180 NV21 only')
    if (args.business_idle or args.business_stress) and not args.business_config: ap.error('Business load flags require config')
    if args.business_idle and args.business_stress: ap.error('Idle and stress are mutually exclusive')
    if args.queue_depth is None:
        args.queue_depth = 4 if args.nv21 or args.direct_venc or args.fullfov_probe else 8 if args.camera_api else 16
    if not 1 <= args.seconds <= 1800:
        ap.error('seconds must be 1..1800')
    if (args.driver_build/'BUILD_INCOMPLETE').exists():
        ap.error('Build incomplete; rebuild successfully before deployment')
    manifest = json.loads((args.driver_build/'manifest.json').read_text())
    official = (manifest.get('profile') or {}).get('status') == 'official_source_test'
    if official != bool(args.official_mode):
        ap.error('Official builds require --official-mode; this flag only accepts official builds')
    if (args.camera_api or args.direct_venc or args.record or args.exercise_switch) and not official:
        ap.error('Camera API recording/switch checks require an official build')
    if args.direct_venc and (args.camera_api or not args.record):
        ap.error('--direct-venc requires --record and is separate from --camera-api')
    if args.record and not (args.camera_api or args.direct_venc):
        ap.error('Recording requires --camera-api or --direct-venc')
    if args.exercise_switch and not args.camera_api:
        ap.error('Switch checks require --camera-api')
    if args.record and args.seconds > 30:
        ap.error('Recording is bounded to 30 seconds; run performance tests separately')
    if official:
        args.experimental = args.hfr = True
        if args.official_mode.startswith('full') and not args.camera_api:
            args.queue_depth = 4
    if args.burst and (not official or args.official_mode!='crop360' or args.seconds!=1 or
                       not args.nv21 or args.camera_api or args.record or args.direct_venc):
        ap.error('--burst requires official crop360, --seconds 1 and --nv21')
    if args.realtime and (not official or args.camera_api):
        ap.error('--realtime supports official direct capture and direct-VENC recording, not Camera API')
    if args.itp_depth and (not official or args.camera_api or args.direct_venc):
        ap.error('--itp-depth currently supports the official direct capture probe')
    if args.venc_copy: args.venc_worker=True
    if not 0 <= args.max_mipi_errors <= 1000:
        ap.error('--max-mipi-errors must be in 0..1000')
    if args.max_mipi_errors and not args.direct_venc:
        ap.error('--max-mipi-errors is valid only with --direct-venc')
    if (args.venc_retry or args.venc_fps or args.venc_worker or args.venc_depth) and not args.direct_venc:
        ap.error('VENC diagnostics require --direct-venc')
    if args.venc_worker and args.official_mode!='crop360':
        ap.error('The worker pool configuration is currently validated only for crop360')
    inferred = (manifest.get('profile') or {}).get('status') == 'reverse_engineered_candidate'
    native = (manifest.get('profile') or {}).get('status') == 'fullfov_timing_candidate'
    if args.fullfov_probe != native or (native and args.queue_depth != 4):
        ap.error('Native full-array probes require --fullfov-probe and queue depth 4')
    if args.crop_probe != inferred:
        ap.error('Inferred crop builds require --crop-probe; this flag only accepts crop builds')
    if args.crop_probe or args.fullfov_probe:
        args.experimental = args.hfr = True
    if (manifest.get('profile') or {}).get('compile_only'):
        ap.error('Synthetic compile-only profiles cannot be deployed')
    if args.hfr and (not args.experimental or not manifest['hfr_available']):
        ap.error('HFR requires an experimental driver built from a supplied vendor profile')
    sys.path.insert(0, str(args.skill.resolve()/'scripts'))
    from maixpy_skill.config import get_device
    from maixpy_skill.ssh import ssh_command, CommandResult, scp_to_command, scp_from_command, command_env
    from maixpy_skill import launcher
    device = get_device()
    stamp = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6]
    local = Path('.maixpy/runs')/('os04a10-'+stamp)
    local.mkdir(parents=True)
    remote = args.remote_root+'/os04a10-'+stamp
    def ssh(command, name, timeout=30, check=True):
        # A quiet 30-minute benchmark needs SSH keepalives. Reuse helper
        # argument/credential handling; never put the password in argv.
        argv=ssh_command(device,command)
        at=argv.index('ssh')+1
        argv[at:at]=['-o','ServerAliveInterval=15','-o','ServerAliveCountMax=4']
        try:
            proc=subprocess.run(argv,env=command_env(device),capture_output=True,text=True,timeout=timeout)
            result=CommandResult(proc.returncode,proc.stdout,proc.stderr)
        except subprocess.TimeoutExpired as exc:
            decode=lambda value: value.decode(errors='replace') if isinstance(value,bytes) else (value or '')
            result=CommandResult(124,decode(exc.stdout),decode(exc.stderr)+'\nHost SSH wait timed out\n')
        (local/(name+'.log')).write_text(result.stdout)
        (local/(name+'.err')).write_text(result.stderr)
        if check and result.returncode:
            raise RuntimeError(f'{name} failed ({result.returncode}); see {local}')
        return result
    def transfer(command):
        result = subprocess.run(command, env=command_env(device), capture_output=True, text=True, timeout=max(60, args.seconds // 10) if args.business_config else 60)
        if result.returncode:
            raise RuntimeError('Transfer failed: '+result.stderr)
    status = ssh(launcher.status_command(), 'mode-before').stdout
    active = ssh("ps -eo args | awk '$1 ~ /^\\/maixapp\\/apps\\// && $1 !~ /^\\/maixapp\\/apps\\/launcher\\// {print $1}'", 'apps-before').stdout.strip()
    restore_camera = active == '/maixapp/apps/camera/camera' and args.pause_camera_app
    if active and not restore_camera:
        raise RuntimeError('Another app is active; stop it before this isolated benchmark: '+active)
    if restore_camera:
        ssh('test ! -e /tmp/run_app.txt', 'no-pending-app-switch')
    before = ssh('sha256sum /opt/lib/libsns_os04a10.so; cat /proc/ax_proc/version', 'baseline-before').stdout
    if manifest['msp_version'] not in before:
        raise RuntimeError('Device MSP does not match this build')
    ssh('mkdir -p '+shlex.quote(args.remote_root)+' && mkdir '+shlex.quote(remote), 'stage')
    program = 'business_capture' if args.business_config else 'direct_record' if args.direct_venc else 'camera_test' if args.camera_api else 'capture'
    files = [(args.driver_build/program, 'capture'),
             (args.sdk/'components/maixcam_lib/lib_maixcam2/libmaixcam_lib.so', 'libmaixcam_lib.so'),
             (args.sdk/'components/nn/lib/libms_asr_ax630c.so', 'libms_asr_ax630c.so'),
             (args.sdk/'components/3rd_party/alsa_lib/lib/maixcam2/libasound.so', 'libasound.so.2'),
             (args.sdk/'components/3rd_party/datachannel/lib/maixcam2/libdatachannel.so', 'libdatachannel.so'),
             (args.sdk/'dl/extracted/onnxruntime_srcs/maixcam2_onnxruntime_v1.22.0/lib/libonnxruntime.so.1.22.0', 'libonnxruntime.so.1'),
             (args.sdk/('dl/extracted/maixcam2_msp_srcs/maixcam2_msp_arm64_glibc_v'+manifest['msp_version'])/'out/arm64_glibc/third-party/lib/libtinyalsa.so.2.0.0', 'libtinyalsa.so.2')]
    if args.business_config: files.append((args.business_config,'business.conf'))
    if args.experimental:
        files.append((args.driver_build/'libsns_os04a10.so', 'libsns_os04a10.so'))
    if args.system_media_lib:
        files = [(source, name) for source, name in files if name != 'libmaixcam_lib.so']
        ssh('sha256sum /usr/lib/libmaixcam_lib.so; readlink -f /usr/lib/libmaixcam_lib.so', 'system-media-library')
    hashes = {}
    def archive_metadata(info):
        info.mtime = 0  # Device clock may not be synchronized with the host.
        return info
    with tarfile.open(local/'stage.tar', 'w') as archive:
        for source, name in files:
            hashes[name] = hashlib.sha256(source.read_bytes()).hexdigest()
            archive.add(source.resolve(), arcname=name, recursive=False, filter=archive_metadata)
    transfer(scp_to_command(device, str((local/'stage.tar').resolve()), remote+'/stage.tar'))
    ssh('cd '+shlex.quote(remote)+' && tar -xf stage.tar && rm -f stage.tar', 'unpack')
    (local/'stage.tar').unlink()
    actual = ssh('cd '+shlex.quote(remote)+' && sha256sum '+ ' '.join(shlex.quote(n) for n in hashes), 'staged-hashes').stdout
    for line in actual.splitlines():
        digest, name = line.split(maxsplit=1)
        if hashes.get(name.strip()) != digest:
            raise RuntimeError('Staged file checksum mismatch')
    env = 'LD_LIBRARY_PATH='+shlex.quote(remote+':/opt/lib:/usr/lib:/lib')
    ssh('chmod +x '+shlex.quote(remote+'/capture')+' && '+env+' ldd '+shlex.quote(remote+'/capture'), 'dependencies')
    if 'not found' in (local/'dependencies.log').read_text():
        raise RuntimeError('Missing device dependency; see dependencies.log')
    # Fast negative case must not initialize SYS or touch the sensor.
    if not manifest['hfr_available']:
        reject = ssh(env+' '+shlex.quote(remote+'/capture')+' --hfr', 'hfr-rejection', check=False)
        if reject.returncode != 3:
            raise RuntimeError('Missing-profile rejection did not return 3')
    entered = False
    result_code = 1
    try:
        entered = True
        ssh(development_enter_command(launcher, args.headless, restore_camera), 'mode-enter', timeout=30)
        command = ['timeout', '--signal=TERM', '--kill-after=5', str(args.seconds+(90 if args.exercise_switch else 20)), './capture', '--seconds', str(args.seconds)]
        if args.gdb:
            at = command.index('./capture')
            command[at:at] = ['gdb', '--batch', '-ex', 'run', '-ex', 'info registers x0 x19 x20 x26', '-ex', 'bt', '--args']
        command += ['--queue-depth', str(args.queue_depth)]
        if args.experimental: command += ['--sensor-lib', remote+'/libsns_os04a10.so']
        if args.hfr: command += ['--hfr']
        if args.sample_raw: command += ['--sample-raw']
        if args.sample_frame: command += ['--sample-frame']
        if args.nv21: command += ['--nv21']
        if args.official_mode: command += ['--mode', args.official_mode]
        if args.record: command += ['--record']
        if args.burst: command += ['--burst']
        if args.realtime: command += ['--realtime']
        if args.max_mipi_errors: command += ['--max-mipi-errors',str(args.max_mipi_errors)]
        if args.itp_depth: command += ['--itp-depth',str(args.itp_depth)]
        if args.venc_retry: command += ['--venc-retry']
        if args.venc_worker: command += ['--venc-worker']
        if args.venc_copy: command += ['--venc-copy']
        if args.venc_fps: command += ['--venc-fps',str(args.venc_fps)]
        if args.venc_depth: command += ['--venc-depth',str(args.venc_depth)]
        if args.business_config: command += ['--business-config',remote+'/business.conf']
        if args.business_idle: command += ['--business-idle']
        if args.business_stress: command += ['--business-stress']
        if args.exercise_switch: command += ['--exercise-switch']
        if args.observe_gaps: command += ['--observe-gaps']
        if args.no_health: command += ['--no-health']
        remote_command='cd '+shlex.quote(remote)+' && '+env+' '+shlex.join(command)
        remote_command+=' > capture.stdout.log 2> capture.stderr.log; status=$?; printf "%s\\n" "$status" > '+shlex.quote(remote+'/process_exit_code')+'; exit "$status"'
        result = ssh(remote_command, 'capture-transport', timeout=args.seconds+(145 if args.exercise_switch else 75), check=False)
        result_code = result.returncode
    finally:
        if entered and ('daemon=running' in status or 'launcher=running' in status):
            if restore_camera:
                # Same /tmp request format as maix::app::switch_app(). Do not
                # alter persistent autostart configuration or launch two UIs.
                ssh("printf '%s\\n' '/maixapp/apps/camera/camera' 'camera' '' > /tmp/run_app.txt", 'request-camera-restore')
            ssh(launcher.exit_command(), 'mode-restore', timeout=30)
        ssh(launcher.status_command(), 'mode-after')
        after = ssh('sha256sum /opt/lib/libsns_os04a10.so; cat /proc/ax_proc/version', 'baseline-after').stdout
        if before != after:
            raise RuntimeError('Original system driver or MSP changed during experiment')
    artifact_names = ['motion.csv','camera_settings.json','vision.csv','targets.jsonl','leases.json','business.json','capture.json', 'frames.csv', 'loaded_maps.txt', 'sample.raw', 'sample.json',
                      'registers_before.csv', 'registers_after.csv', 'health.csv', 'progress.json', 'progress.jsonl', 'frames.csv.io.json', 'progress.jsonl.io.json', 'targets.jsonl.io.json', 'vision.csv.io.json', 'motion.csv.io.json', 'sample.nv21',
                      'capture.stdout.log','capture.stderr.log','process_exit_code',
                      'record.h264','encoded_frames.csv','api_checks.csv','ivps_config.csv','sample.nv12','record.nv21',
                      'scheduler.txt','venc_config.json','venc_send.csv','itp_depth.json']
    artifact_names += [name+'_'+suffix+'.txt' for name in ['sensor_info','vin_attr','vin_statistics','mipi_rx_attr','mipi_rx_status']
                       for suffix in ['before','after']]
    names = ssh('cd '+shlex.quote(remote)+' && for n in '+shlex.join(artifact_names)+
                '; do if test -f "$n"; then echo "$n"; fi; done', 'artifact-list').stdout.splitlines()
    if any(n not in artifact_names for n in names):
        raise RuntimeError('Unexpected artifact filename')
    if names:
        ssh('cd '+shlex.quote(remote)+' && tar -cf artifacts.tar '+shlex.join(names), 'pack-artifacts', timeout=120 if args.business_config and args.seconds>120 else 30)
        transfer(scp_from_command(device, remote+'/artifacts.tar', str(local/'artifacts.tar')))
        with tarfile.open(local/'artifacts.tar') as archive:
            for member in archive:
                if member.name not in artifact_names or not member.isfile():
                    raise RuntimeError('Unexpected archive member')
                with archive.extractfile(member) as source, (local/member.name).open('wb') as destination:
                    import shutil
                    shutil.copyfileobj(source, destination)
        (local/'artifacts.tar').unlink()
    exit_path = local/'process_exit_code'
    process_code = int(exit_path.read_text().strip()) if exit_path.exists() else None
    transport_code = result_code
    # Preserve transport failures even when capture completed successfully.
    if result_code == 0 and process_code != 0:
        result_code = process_code if process_code is not None else 1
    if result_code == 0 and not (local/'capture.json').exists():
        result_code = 1  # A debugger exit or missing measurement is not a successful capture.
    (local/'run.json').write_text(json.dumps({'device':device.public_dict(), 'remote_dir':remote,
        'experimental':args.experimental, 'hfr':args.hfr, 'crop_probe':args.crop_probe,
        'fullfov_probe':args.fullfov_probe,
        'official_mode':args.official_mode, 'camera_api':args.camera_api, 'record':args.record,
        'direct_venc':args.direct_venc, 'business':bool(args.business_config), 'business_idle':args.business_idle, 'business_stress':args.business_stress,
        'burst':args.burst,
        'realtime':args.realtime,
        'max_mipi_errors':args.max_mipi_errors,
        'venc_retry':args.venc_retry,'venc_fps':args.venc_fps,
        'venc_depth':args.venc_depth,
        'venc_worker':args.venc_worker,
        'venc_copy':args.venc_copy,
        'itp_depth':args.itp_depth,
        'system_media_lib':args.system_media_lib, 'no_health':args.no_health, 'observe_gaps':args.observe_gaps,
        'paused_official_camera':restore_camera, 'headless':args.headless,
        'profile':manifest.get('profile'), 'nv21':args.nv21, 'seconds':args.seconds,
        'exit_code':result_code, 'transport_exit_code':transport_code,
        'process_exit_code':process_code, 'queue_depth':args.queue_depth,
        'staged_sha256':hashes},indent=2)+'\n')
    ssh('cd '+shlex.quote(remote)+' && rm -f '+shlex.join(list(hashes)+['artifacts.tar']), 'clean-staged-binaries')
    print(local)
    if (local/'capture.stdout.log').exists():
        print((local/'capture.stdout.log').read_text()[-4000:])
    if (local/'capture.stderr.log').exists():
        print((local/'capture.stderr.log').read_text()[-2000:])
    return result_code


if __name__ == '__main__':
    raise SystemExit(main())
