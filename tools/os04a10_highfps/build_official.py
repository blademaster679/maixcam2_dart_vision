#!/usr/bin/env python3
"""Build fixed official OS04A10/CDK sources against the device-matched MSP in isolation."""
import argparse,difflib,hashlib,json,shlex,shutil,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent
MSP='3.0.0_20250319114413'
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=Path('.maixpy/official-build'));ap.add_argument('--app-only',action='store_true');ap.add_argument('--business',action='store_true');a=ap.parse_args()
 out=a.out.resolve();sdk=Path.home()/'maix/MaixCDK';build=Path('build').resolve();archive=Path('.maixpy/official-comparison').resolve()
 source_inputs=[*HERE.glob('*.cpp'),*HERE.glob('*.hpp'),*Path('main/src').glob('*.cpp'),*Path('main/include/dart').glob('*.hpp')]
 input_hashes={str(p.resolve()):sha(p) for p in source_inputs}
 flags={k:shlex.split(v) for line in (build/'main/CMakeFiles/main.dir/flags.make').read_text().splitlines() if ' = ' in line for k,v in [line.split(' = ',1)]}
 original=shlex.split((build/'CMakeFiles/dart_green_detect.dir/link.txt').read_text());cxx=original[0]
 if not a.app_only:
  if out.exists(): raise SystemExit('Choose a new output directory')
  out.mkdir(parents=True)
  for item in json.loads((archive/'fetch.json').read_text()):
   p=Path(item['file']);assert sha(p)==item['sha256'],str(p)
  msp=sdk/f'dl/extracted/maixcam2_msp_srcs/maixcam2_msp_arm64_glibc_v{MSP}'
  for part in ['ov_os04a10','common','i2c','mipi_switch','include']: shutil.copytree(msp/'component/isp_proton/sensor'/part,out/'sensor'/part)
  sensor=out/'sensor/ov_os04a10';up=archive/'msp/component/isp_proton/sensor/ov_os04a10'
  for p in up.glob('*'):
   if p.suffix in ['.c','.h'] or p.name=='Makefile.dynamic': shutil.copy2(p,sensor/p.name)
  (out/'sensor/include/ax_module_version.h').write_text('#define AXERA_MODULE_VERSION "official-71ca5afe-isolated-test"\n')
  inc=[msp/'out/arm64_glibc/include',out/'sensor/include',out/'sensor/common/include',out/'sensor/i2c',out/'sensor/mipi_switch',sensor/'params_file/ax620e']
  sources=[p for p in sorted(sensor.glob('*.c')) if p.name!='os04a10_qs.c']
  for sub in ['common/src','i2c','mipi_switch']: sources+=sorted((out/'sensor'/sub).glob('*.c'))
  gcc=cxx.removesuffix('g++')+'gcc'
  cmd=[gcc,'-shared','-fPIC','-O2','-DLINUX','-DUSE_DEFAULT_PARAM','-fvisibility=hidden',*['-I'+str(p) for p in inc],*map(str,sources),'-Wl,--no-undefined','-Wl,-soname,libsns_os04a10.so','-L'+str(msp/'out/arm64_glibc/lib'),'-lax_sys','-lpthread','-lm','-o',str(out/'libsns_os04a10.so')]
  with (out/'sensor-build.log').open('w') as log: subprocess.run(cmd,check=True,stdout=log,stderr=subprocess.STDOUT)
  for rel in ['components/vision/include/maix_camera.hpp','components/vision/port/maixcam2/maix_camera_maixcam2.cpp','components/maixcam_lib/include/maixcam2/ax_middleware.hpp']:
   shutil.copy2(archive/'cdk'/rel,out/Path(rel).name)
  # Only harness isolation edits: disable AI-ISP pool selection and bind the explicitly preloaded official object.
  p=out/'ax_middleware.hpp';old=p.read_text();s=old
  s=s.replace('app::get_sys_config_kv("npu", "ai_isp", "1") == "1" ? AX_TRUE : AX_FALSE','AX_FALSE')
  target='        return COMMON_ISP_GetSnsObj(eSnsType);'
  assert target in s
  s=s.replace(target,'''        if (eSnsType == OMNIVISION_OS04A10) {
            auto *obj = reinterpret_cast<AX_SENSOR_REGISTER_FUNC_T *>(dlsym(RTLD_DEFAULT, "gSnsos04a10Obj"));
            err::check_null_raise(obj, "isolated official sensor object unavailable");
            return obj;
        }
'''+target)
  s='#include <dlfcn.h>\n'+s;p.write_text(s)
  (out/'harness-isolation.patch').write_text(''.join(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile='official/ax_middleware.hpp',tofile='test/ax_middleware.hpp')))
  manifest={'msp_version':MSP,'hfr_available':True,'profile':{'status':'official_source_test','sensor':'OS04A10','source_commits':{'msp':'71ca5afe0b7db721c1c95a7488db6d9223060365','cdk':'fa498da900ffced8a09d79793b43d07f5bacf65e'}},'library_sha256':sha(out/'libsns_os04a10.so'),'build_command':cmd,'sensor_source_sha256':{p.name:sha(p) for p in sensor.glob('*.[ch]')},'scope':'Official changed sensor and Camera sources with local device-matched SDK dependencies; isolated object binding/AI-ISP harness patch; no system replacement'}
  (out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
 (out/'BUILD_INCOMPLETE').write_text('Application compilation in progress or failed; do not deploy.\n')
 # Compile the official Camera translation unit and explicitly link it before the old SDK archives.
 # Quoted includes in this local SDK header must resolve to the same Camera class definition.
 shutil.copy2(sdk/'components/vision/include/maix_video.hpp',out/'maix_video.hpp')
 compiles=[]
 for source,object_name in [(out/'maix_camera_maixcam2.cpp','official_camera.o'),(HERE/'official_capture.cpp','capture.o'),(HERE/'official_camera_test.cpp','camera_test.o'),(HERE/'official_record.cpp','direct_record.o')]:
  if not source.exists(): continue
  cmd=[cxx,'-I'+str(out),*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-g','-c',str(source),'-o',str(out/object_name)]
  with (out/(object_name+'.log')).open('w') as log: subprocess.run(cmd,check=True,stdout=log,stderr=subprocess.STDOUT)
  compiles.append(cmd)
 if a.business:
  sources=[HERE/'business_capture.cpp',*[Path('main/src').resolve()/n for n in ['config.cpp','green_detector.cpp','green_detector_core.cpp','target_fusion.cpp','target_json.cpp','visual_motion.cpp','nv21_pipeline.cpp']]]
  for source in sources:
   obj=out/('business_'+source.stem+'.o')
   cmd=[cxx,'-I'+str(out),*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],'-O2','-g','-c',str(source),'-o',str(obj)]
   with (out/(obj.name+'.log')).open('w') as log: subprocess.run(cmd,check=True,stdout=log,stderr=subprocess.STDOUT)
   compiles.append(cmd)
  link=[x for x in original if x not in ['CMakeFiles/dart_green_detect.dir/exe_src.c.o','main/libmain.a']]
  link.insert(1,'-Wl,--as-needed');i=link.index('-o');link[i+1]=str(out/'business_capture');link[i:i]=[str(out/('business_'+source.stem+'.o')) for source in sources]
  with (out/'business-link.log').open('w') as log: subprocess.run(link,cwd=build,check=True,stdout=log,stderr=subprocess.STDOUT)
  (out/'business-sources.json').write_text(json.dumps({str(p):sha(p) for p in [*sources,HERE/'official_capture.cpp',HERE/'vin_nv21_frame.hpp',HERE/'vin_camera_settings.hpp',*Path('main/include/dart').glob('*.hpp')]},indent=2)+'\n')
 for name,objects in [('capture',['capture.o']),('camera_test',['camera_test.o','official_camera.o']),('direct_record',['direct_record.o'])]:
  if not (out/objects[0]).exists(): continue
  link=[x for x in original if x not in ['CMakeFiles/dart_green_detect.dir/exe_src.c.o','main/libmain.a']]
  link.insert(1,'-Wl,--as-needed');i=link.index('-o');link[i+1]=str(out/name);link[i:i]=[str(out/o) for o in objects]
  with (out/(name+'-link.log')).open('w') as log: subprocess.run(link,cwd=build,check=True,stdout=log,stderr=subprocess.STDOUT)
 app_sources={}
 for name in ['official_capture.cpp','official_camera_test.cpp','official_record.cpp','capture_health.hpp','direct_venc.hpp','durable_checkpoint.hpp','frame_continuity.hpp','vin_venc_queue.hpp','venc_input_copy.hpp','vin_camera_settings.hpp']:
  if (HERE/name).exists():
   shutil.copy2(HERE/name,out/name);app_sources[name]=sha(HERE/name)
 if any(sha(Path(p))!=digest for p,digest in input_hashes.items()):
  raise SystemExit('Source changed during build; BUILD_INCOMPLETE retained. Rebuild before deployment.')
 (out/'app-build.json').write_text(json.dumps({'compile':compiles,'app_source_sha256':app_sources,'official_camera_sha256':sha(out/'maix_camera_maixcam2.cpp'),'middleware_sha256':sha(out/'ax_middleware.hpp'),'local_sdk_commit':subprocess.check_output(['git','-C',str(sdk),'rev-parse','HEAD'],text=True).strip()},indent=2)+'\n');(out/'BUILD_INCOMPLETE').unlink();print(out)
if __name__=='__main__': main()
