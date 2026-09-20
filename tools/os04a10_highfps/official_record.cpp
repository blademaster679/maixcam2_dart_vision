// Official sensor NV12 recording; asynchronous AX VENC and input/encoded metadata.
#include "ax_middleware.hpp"
// Runtime settings for the exact official driver; no synthesized register table.
#define OS04A10_HFR_AVAILABLE 1
#define OS04A10_KEEP_RECEIVER 1
#define OS04A10_HFR_BAYER AX_BP_RGGB
static int OS04A10_HFR_WIDTH=1344, OS04A10_HFR_HEIGHT=760, OS04A10_HFR_FPS=60;

#include <chrono>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <dlfcn.h>
#include <sstream>
#include <memory>
#include <cmath>
#include <sched.h>
#include <sys/statvfs.h>
#include "capture_health.hpp"
#include "direct_venc.hpp"
#include "frame_continuity.hpp"
#include "vin_venc_queue.hpp"
#include "vin_camera_settings.hpp"

using namespace maix::middleware::maixcam2;
static volatile sig_atomic_t stopped = 0;
static void stop_handler(int) { stopped = 1; }
static uint64_t now_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct Record { uint64_t seq, pts, mono; uint32_t width, height, stride, size; int format; };
struct Health { uint64_t elapsed_us; double temperature; long mipi_errors, rss_kb; };
static Health health(AX_SENSOR_REGISTER_FUNC_T *sns, uint64_t elapsed) {
    Health h{elapsed, -999, -1, -1};
    AX_U32 hi=0,lo=0;
    if (sns->pfn_sensor_read_register && !sns->pfn_sensor_read_register(0,0x4f06,&hi)
        && !sns->pfn_sensor_read_register(0,0x4f07,&lo)) {
        unsigned raw=(hi<<8)|lo;
        if (raw) {
            unsigned magnitude=raw>0xc000 ? raw-0xc000 : raw;
            h.temperature=((magnitude>>8)+(magnitude&255)/255.0)*(raw>0xc000 ? -1 : 1);
        }
    }
    std::ifstream status("/proc/ax_proc/mipi_rx/status");
    std::string line;
    while (std::getline(status,line)) {
        if (line.find("PhyStatus")!=std::string::npos && std::getline(status,line)) {
            std::istringstream values(line);
            std::string phy; long resets,errors;
            if (values>>phy>>resets>>errors) h.mipi_errors=errors;
        }
    }
    std::ifstream process("/proc/self/status");
    while (std::getline(process,line)) {
        if (line.rfind("VmRSS:",0)==0) {
            std::istringstream values(line.substr(6)); values>>h.rss_kb;
        }
    }
    return h;
}
static void save_proc(const char *suffix) {
    for (const char *name : {"sensor/info", "vin/attr", "vin/statistics", "mipi_rx/attr", "mipi_rx/status"}) {
        std::ifstream in(std::string("/proc/ax_proc/") + name);
        std::string filename(name);
        std::replace(filename.begin(), filename.end(), '/', '_');
        std::ofstream out(filename + suffix);
        if (in) out << in.rdbuf();
    }
}
static void save_registers(AX_SENSOR_REGISTER_FUNC_T *sns, const char *filename) {
    std::ofstream out(filename);
    out << "address,value,result\n";
    std::vector<unsigned> addresses = {0x0303,0x0304,0x0305,0x0306,0x0307,0x0308,0x0309,0x030c,
        0x3714,0x37cf,0x4009,0x4051,0x4601,0x4603,0x460c,0x3426,0x3427,0x3428,0x4800,0x4837,0x0322,0x0323,0x0324,0x0325,0x0328,0x032a,0x032f,0x3501,0x3502,0x384c,0x384d,0x4f06,0x4f07};
    for (unsigned a=0x3800; a<=0x3822; ++a) addresses.push_back(a);
    for (auto address : addresses) {
        AX_U32 value=0;
        const int rc = sns->pfn_sensor_read_register ? sns->pfn_sensor_read_register(0,address,&value) : -1;
        out << address << ',' << value << ',' << rc << '\n';
    }
}
static bool parse_wb_gains(const char *text, std::array<float,4> &gains) {
    std::istringstream input(text ? text : "");
    std::string field;
    for (size_t index=0; index<gains.size(); ++index) {
        if (!std::getline(input,field,',')) return false;
        char *end=nullptr;
        const float value=std::strtof(field.c_str(),&end);
        if (!end || *end || !std::isfinite(value) || value<0.0F || value>1.0F) return false;
        gains[index]=value;
    }
    return !std::getline(input,field,',');
}
int main(int argc, char **argv) {
    int seconds = 10;
    bool hfr = true;
    bool sample_raw = false;
    bool nv21 = true;
    bool record_video = false;
    bool venc_retry = false;
    unsigned venc_fps = 0;
    unsigned venc_depth = 4;
    bool venc_worker = false;
    bool venc_copy = false;
    bool realtime = false;
    long max_mipi_errors = 0;
    int queue_depth = 0;
    const char *sensor_library = nullptr;
    dart::CameraSettings camera_settings;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--mode") && i+1<argc) {
            std::string mode=argv[++i];
            if (mode=="full60") { OS04A10_HFR_WIDTH=1344; OS04A10_HFR_HEIGHT=760; OS04A10_HFR_FPS=60; }
            else if (mode=="full120") { OS04A10_HFR_WIDTH=1344; OS04A10_HFR_HEIGHT=760; OS04A10_HFR_FPS=120; }
            else if (mode=="full180") { OS04A10_HFR_WIDTH=1344; OS04A10_HFR_HEIGHT=760; OS04A10_HFR_FPS=180; }
            else if (mode=="crop240") { OS04A10_HFR_WIDTH=640; OS04A10_HFR_HEIGHT=360; OS04A10_HFR_FPS=240; }
            else if (mode=="crop360") { OS04A10_HFR_WIDTH=640; OS04A10_HFR_HEIGHT=360; OS04A10_HFR_FPS=360; }
            else return 2;
        }
        else if (!std::strcmp(argv[i], "--hfr")) hfr = true;
        else if (!std::strcmp(argv[i], "--sample-raw")) sample_raw = true;
        else if (!std::strcmp(argv[i], "--sample-frame")) sample_raw = true;
        else if (!std::strcmp(argv[i], "--nv21")) nv21 = true;
        else if (!std::strcmp(argv[i], "--record")) record_video = true;
        else if (!std::strcmp(argv[i], "--venc-retry")) venc_retry = true;
        else if (!std::strcmp(argv[i], "--venc-worker")) venc_worker = true;
        else if (!std::strcmp(argv[i], "--venc-copy")) {venc_copy=true;venc_worker=true;}
        else if (!std::strcmp(argv[i], "--realtime")) realtime = true;
        else if (!std::strcmp(argv[i], "--max-mipi-errors") && i+1 < argc) {
            char *end=nullptr;
            const long value=std::strtol(argv[++i],&end,10);
            if (*end || value<0 || value>1000) return 2;
            max_mipi_errors=value;
        }
        else if (!std::strcmp(argv[i], "--venc-depth") && i+1<argc) {
            const std::string value=argv[++i];
            if(value!="4" && value!="8")return 2;
            venc_depth=std::stoul(value);
        }
        else if (!std::strcmp(argv[i], "--venc-fps") && i+1<argc) {
            const std::string value=argv[++i];
            if(value!="180" && value!="360") return 2;
            venc_fps=std::stoul(value);
        }
        else if (!std::strcmp(argv[i], "--queue-depth") && i+1<argc) {
            char *end=nullptr;
            long value=std::strtol(argv[++i],&end,10);
            if (*end || value<4 || value>32) return 2;
            queue_depth=static_cast<int>(value);
        }
        else if (!std::strcmp(argv[i], "--sensor-lib") && i+1 < argc) sensor_library = argv[++i];
        else if (!std::strcmp(argv[i], "--exposure-us") && i+1 < argc) {
            char *end=nullptr;
            const long value=std::strtol(argv[++i],&end,10);
            if (*end || value<1 || value>1000000) return 2;
            camera_settings.exposure_us=static_cast<int>(value);
        }
        else if (!std::strcmp(argv[i], "--gain") && i+1 < argc) {
            char *end=nullptr;
            const long value=std::strtol(argv[++i],&end,10);
            if (*end || value<0 || value>65535) return 2;
            camera_settings.gain=static_cast<int>(value);
        }
        else if (!std::strcmp(argv[i], "--wb-gains") && i+1 < argc) {
            camera_settings.manual_white_balance=true;
            if (!parse_wb_gains(argv[++i],camera_settings.white_balance_gain)) return 2;
        }
        else if (!std::strcmp(argv[i], "--auto-wb")) camera_settings.manual_white_balance=false;
        else if (!std::strcmp(argv[i], "--seconds") && i+1 < argc) {
            char *end = nullptr;
            long value = std::strtol(argv[++i], &end, 10);
            if (*end || value < 1 || value > 600) return 2;
            seconds = static_cast<int>(value);
        } else { std::cerr << "usage: direct_record --mode full180 --record --sensor-lib path [--seconds 1..600] [--exposure-us N] [--gain N] [--wb-gains R,Gr,Gb,B|--auto-wb] [--sample-frame] [--realtime] [--max-mipi-errors N]\n"; return 2; }
    }
    camera_settings.width=OS04A10_HFR_WIDTH;
    camera_settings.height=OS04A10_HFR_HEIGHT;
    camera_settings.fps=OS04A10_HFR_FPS;
    if (!record_video || seconds>600) return 2;
    if (!queue_depth) queue_depth = nv21 ? 4 : 16;
    if (hfr && OS04A10_HFR_WIDTH > 640 && queue_depth != 4) {
        std::cerr << "Full-array probes require --queue-depth 4 with vendor-sized pools\n";
        return 2;
    }
#if !OS04A10_HFR_AVAILABLE
    if (hfr) {
        std::cerr << "360fps unavailable: vendor binning register sequence has not been supplied\n";
        return 3; // Before sensor discovery, SYS init or register access.
    }
#endif
    std::signal(SIGINT, stop_handler);
    std::signal(SIGTERM, stop_handler);
    std::vector<Record> records;
    records.reserve(static_cast<size_t>(seconds + 1) * 400);
    size_t acquire_errors = 0, release_errors = 0, worker_queue_rejected = 0;
    int rc = 0;
    bool storage_low = false;
    try {
        if (!sensor_library) return 2;
        void *official_handle=dlopen(sensor_library,RTLD_NOW|RTLD_GLOBAL);
        using CropFn=int(*)(int,unsigned,unsigned,unsigned,unsigned,float);
        auto crop=official_handle ? reinterpret_cast<CropFn>(dlsym(official_handle,"os04a10_set_crop")) : nullptr;
        if (!crop) { std::cerr << "Official crop API unavailable\n"; return 4; }
        unsigned x=OS04A10_HFR_WIDTH==640 ? 704 : 0, y=OS04A10_HFR_WIDTH==640 ? 404 : 4;
        if (crop(0,x,y,OS04A10_HFR_WIDTH,OS04A10_HFR_HEIGHT,OS04A10_HFR_FPS)) return 4;
        if (hfr) {
            // Size the dedicated process pools for the actual sensor output.
            // Leave 8 buffers beyond the dump/consumer queue for ISP work.
            for (auto &pool : gtPrivatePoolSingleOs04a10Sdr) {
                pool.nWidth=pool.nWidthStride=OS04A10_HFR_WIDTH; pool.nHeight=OS04A10_HFR_HEIGHT;
                pool.nBlkCnt=queue_depth+8;
            }
            for (auto &pool : gtSysCommPoolSingleOs04a10Sdr) {
                pool.nWidth=pool.nWidthStride=OS04A10_HFR_WIDTH; pool.nHeight=OS04A10_HFR_HEIGHT;
                if (pool.enCompressMode==AX_COMPRESS_MODE_NONE) pool.nBlkCnt=queue_depth+8+(venc_worker ? 32 : 0);
            }
        } else {
            queue_depth=4; // Full-resolution baseline retains vendor pool sizing.
        }
        SYS sys(true);
        if (sys.init() != maix::err::ERR_NONE) return 4;
        VI vi;
        const auto sensor = vi.get_sensor_name();
        if (!sensor.first || sensor.second != "os04a10") {
            std::cerr << "OS04A10 required, detected " << sensor.second << '\n'; return 4;
        }
        COMMON_SYS_ARGS_T common = {}, priv = {};
        SAMPLE_VIN_PARAM_T param = {};
        param.eSysCase = SAMPLE_VIN_SINGLE_OS04A10;
        param.eSysMode = COMMON_VIN_SENSOR;
        param.eHdrMode = AX_SNS_LINEAR_MODE;
        param.eLoadRawNode = LOAD_RAW_IFE;
        param.bAiispEnable = AX_FALSE;
        param.bSensorCrop=AX_TRUE;
        param.nSensorCropX=x; param.nSensorCropY=y;
        param.nSensorCropW=param.nSensorWidth=OS04A10_HFR_WIDTH;
        param.nSensorCropH=param.nSensorHeight=OS04A10_HFR_HEIGHT;
        param.nSensorFps=OS04A10_HFR_FPS;
        if (vi.config_sample_case(&param, &common, &priv)) return 4;
        auto &module = AxModuleParam::getInstance();
        module.lock(AX_MOD_VI);
        auto *vp = static_cast<ax_vi_mod_t *>(module.get_param(AX_MOD_VI));
        auto &cam = vp->cams[0];
        if (sensor_library) {
            // Keep the handle alive until process exit; callbacks outlive this block.
            void *handle = dlopen(sensor_library, RTLD_NOW | RTLD_LOCAL);
            auto *object = handle ? static_cast<AX_SENSOR_REGISTER_FUNC_T *>(dlsym(handle, "gSnsos04a10Obj")) : nullptr;
            if (!object) {
                module.unlock(AX_MOD_VI);
                std::cerr << "Cannot load experimental sensor object: " << dlerror() << '\n';
                return 4;
            }
            cam.ptSnsHdl[0] = object;
            std::cout << "sensor_object=" << sensor_library << '\n';
        }
        cam.tPipeAttr[0].tCompressInfo = {AX_COMPRESS_MODE_NONE, 0};
        if (nv21) {
            cam.tChnAttr[0].eImgFormat = AX_FORMAT_YUV420_SEMIPLANAR;
            cam.tChnAttr[0].tCompressInfo = {AX_COMPRESS_MODE_NONE, 0};
            cam.tChnAttr[0].nDepth = queue_depth;
        }
#if OS04A10_HFR_AVAILABLE
        if (hfr) {
            cam.tSnsAttr.nWidth = OS04A10_HFR_WIDTH; cam.tSnsAttr.nHeight = OS04A10_HFR_HEIGHT;
            cam.tSnsAttr.fFrameRate = OS04A10_HFR_FPS; cam.tSnsAttr.eBayerPattern = OS04A10_HFR_BAYER;
#if !OS04A10_KEEP_RECEIVER
            cam.tMipiAttr.nDataRate = OS04A10_HFR_LANE_MBPS;
#endif
            for (auto &r : cam.tDevAttr.tDevImgRgn) r = {0, 0, OS04A10_HFR_WIDTH, OS04A10_HFR_HEIGHT};
            cam.tDevAttr.eBayerPattern = OS04A10_HFR_BAYER;
            cam.tPipeAttr[0].tPipeImgRgn = {0, 0, OS04A10_HFR_WIDTH, OS04A10_HFR_HEIGHT};
            cam.tPipeAttr[0].nWidthStride = OS04A10_HFR_WIDTH;
            cam.tPipeAttr[0].eBayerPattern = OS04A10_HFR_BAYER;
            cam.tChnAttr[0].nWidth = OS04A10_HFR_WIDTH; cam.tChnAttr[0].nHeight = OS04A10_HFR_HEIGHT;
            cam.tChnAttr[0].nWidthStride = OS04A10_HFR_WIDTH;
        }
#endif
        const unsigned expected_w = cam.tSnsAttr.nWidth, expected_h = cam.tSnsAttr.nHeight;
        module.unlock(AX_MOD_VI);
        if (vi.init() != maix::err::ERR_NONE) return 4;
        Dl_info selected = {};
        if (!cam.ptSnsHdl[0] || !dladdr(reinterpret_cast<void *>(cam.ptSnsHdl[0]->pfn_sensor_chipid), &selected)
            || !selected.dli_fname || (sensor_library && std::strcmp(sensor_library, selected.dli_fname))) {
            std::cerr << "Selected sensor callback library does not match request\n"; return 4;
        }
        std::cout << "active_sensor_callback=" << selected.dli_fname << '\n';
        AX_S32 chip_id = 0;
        if (!cam.ptSnsHdl[0] || !cam.ptSnsHdl[0]->pfn_sensor_chipid ||
            cam.ptSnsHdl[0]->pfn_sensor_chipid(0, &chip_id) != 0 || chip_id != 0x530441) {
            std::cerr << "Chip-ID verification failed\n"; return 4;
        }
        std::cout << "chip_id=0x" << std::hex << chip_id << std::dec
                  << " expected_raw=" << expected_w << 'x' << expected_h << '\n';
        apply_vin_camera_settings(camera_settings,cam.ptSnsHdl[0]);
        std::ifstream maps("/proc/self/maps");
        std::ofstream("loaded_maps.txt") << maps.rdbuf();
        AX_VIN_DUMP_ATTR_T dump = {};
        dump.bEnable = AX_TRUE; dump.nDepth = queue_depth;
        if (!nv21 && AX_VIN_SetPipeDumpAttr(0, AX_VIN_PIPE_DUMP_NODE_IFE, AX_VIN_DUMP_QUEUE_TYPE_DEV, &dump)) return 4;
        save_proc("_before.txt");
        save_registers(cam.ptSnsHdl[0], "registers_before.csv");
        DirectVenc encoder(expected_w,expected_h,OS04A10_HFR_FPS,venc_retry,venc_fps,venc_depth,static_cast<size_t>(seconds + 1) * 400,200000,30000);
        std::unique_ptr<VinVencQueue> sender;
        if(venc_worker)sender=std::make_unique<VinVencQueue>(encoder,venc_copy);
        const int original_policy=sched_getscheduler(0);
        sched_param original_priority={};
        if(original_policy<0 || sched_getparam(0,&original_priority))
            throw std::runtime_error("Cannot read acquisition scheduler");
        if(realtime) {
            sched_param priority={};priority.sched_priority=10;
            if(sched_setscheduler(0,SCHED_FIFO,&priority))
                throw std::runtime_error("Cannot set acquisition SCHED_FIFO");
        }
        std::ofstream("scheduler.txt") << "policy=" << sched_getscheduler(0)
            << " priority=" << (realtime ? 10 : original_priority.sched_priority)
            << " realtime_requested=" << realtime << '\n';
        const auto start = now_us();
        const auto measurement_start = start + 2000000;
        const auto end = measurement_start + static_cast<uint64_t>(seconds) * 1000000;
        unsigned consecutive_errors = 0;
        bool sampled = false;
        std::vector<Health> health_records;
        health_records.reserve(seconds+2);
        auto next_health = measurement_start;
        dart::FrameContinuity continuity;
        dart::CaptureHealthPolicy health_policy(max_mipi_errors);
        while (!stopped && !maix::app::need_exit() && !encoder.failed && (!sender || !sender->failed) && now_us() < end) {
            AX_IMG_INFO_T img = {};
            const auto result = nv21 ? AX_VIN_GetYuvFrame(0, AX_VIN_CHN_ID_MAIN, &img, 200) :
                AX_VIN_GetRawFrame(0, AX_VIN_PIPE_DUMP_NODE_IFE, AX_SNS_HDR_FRAME_L, &img, 200);
            if (result) {
                ++acquire_errors;
                if (++consecutive_errors >= 5) { rc = 5; break; }
                continue;
            }
            consecutive_errors = 0;
            const auto stamp = now_us();
            const auto &f = img.tFrameInfo.stVFrame;
            const Record record{f.u64SeqNum, f.u64PTS, stamp, f.u32Width, f.u32Height,
                                f.u32PicStride[0], f.u32FrameSize, static_cast<int>(f.enImgFormat)};
            // One diagnostic image during warmup; no storage I/O in measurement.
            if (sample_raw && !sampled && stamp >= start + 1000000 && stamp < measurement_start) {
                if (f.u32FrameSize && f.u32FrameSize <= 16*1024*1024) {
                    void *pixels = AX_SYS_Mmap(f.u64PhyAddr[0], f.u32FrameSize);
                    if (pixels) {
                        std::ofstream raw(nv21 ? "sample.nv12" : "sample.raw", std::ios::binary);
                        raw.write(static_cast<const char *>(pixels), f.u32FrameSize);
                        sampled = raw.good();
                        AX_SYS_Munmap(pixels, f.u32FrameSize);
                        std::ofstream meta("sample.json");
                        meta << "{\"width\":" << record.width << ",\"height\":" << record.height
                             << ",\"stride\":" << record.stride << ",\"frame_bytes\":" << record.size
                             << ",\"format\":" << record.format << ",\"sequence\":" << record.seq << "}\n";
                    }
                }
            }
            int send_result=0;
            bool transferred=false;
            if(stamp>=measurement_start && stamp<end) {
                if(sender) {transferred=sender->push(img);if(!transferred) {send_result=AX_ERR_VENC_QUEUE_FULL;++worker_queue_rejected;}}
                else send_result=encoder.send(img.tFrameInfo);
            }
            const auto released = transferred ? 0 : nv21 ? AX_VIN_ReleaseYuvFrame(0, AX_VIN_CHN_ID_MAIN, &img) :
                AX_VIN_ReleaseRawFrame(0, AX_VIN_PIPE_DUMP_NODE_IFE, AX_SNS_HDR_FRAME_L, &img);
            if (released) { ++release_errors; rc = 5; break; }
            if(send_result) {std::cerr<<"VENC send error="<<send_result<<'\n';rc=12;break;}
            if (record.width != expected_w || record.height != expected_h) { rc = 6; break; }
            if (nv21 && record.format != AX_FORMAT_YUV420_SEMIPLANAR) { rc = 6; break; }
            if (stamp >= measurement_start && stamp < end) {
                if (records.size() == records.capacity()) { rc = 7; break; }
                continuity.observe(record.seq,record.pts);
                records.push_back(record);
                // A forward sequence gap means the application missed input frames,
                // but every retained frame and encoded packet is still valid. Keep
                // recording and report the exact loss. Duplicate/backward sequence
                // numbers or non-increasing PTS remain fatal ordering faults.
                if(continuity.fatal()) {rc=10;break;}
                if (stamp>=next_health) {
                    auto state=health(cam.ptSnsHdl[0],stamp-measurement_start);
                    health_records.push_back(state);
                    struct statvfs disk = {};
                    if (statvfs(".", &disk) != 0 ||
                        static_cast<uint64_t>(disk.f_bavail) * disk.f_frsize < 512ULL * 1024 * 1024) {
                        storage_low = true;
                        std::cerr << "Recording stopped: free space below 512 MiB or query failed\n";
                        rc = 13;
                        break;
                    }
                    health_policy.observe(state.temperature,state.mipi_errors);
                    next_health=stamp+1000000;
                    // Datasheet operating junction ceiling is 85 C; stop with 5 C headroom.
                    if (health_policy.fatal()) { rc=9; break; }
                    if (health_records.size()%10==0) {
                        std::ofstream progress("progress.json");
                        progress << "{\"elapsed_s\":" << state.elapsed_us/1e6
                            << ",\"frames\":" << records.size() << ",\"temperature_c\":" << state.temperature
                            << ",\"mipi_errors\":" << state.mipi_errors << ",\"rss_kb\":" << state.rss_kb << "}\n";
                    }
                }
            }
        }
        if(realtime && sched_setscheduler(0,original_policy,&original_priority)) rc=11;
        if(sender) {sender->finish();release_errors+=sender->release_errors;
            if(sender->failed || sender->accepted!=sender->released)rc=12;}
        encoder.finish();
        if(sender && sender->close_pool())rc=12;
        if(encoder.failed || encoder.submitted!=records.size() || encoder.packets!=records.size())rc=12;
        const auto final_health=health(cam.ptSnsHdl[0],now_us()-measurement_start);
        health_records.push_back(final_health);
        health_policy.observe(final_health.temperature,final_health.mipi_errors);
        if(!rc && health_policy.fatal())rc=9;
        save_proc("_after.txt");
        save_registers(cam.ptSnsHdl[0], "registers_after.csv");
        dump.bEnable = AX_FALSE;
        if (!nv21 && AX_VIN_SetPipeDumpAttr(0, AX_VIN_PIPE_DUMP_NODE_IFE, AX_VIN_DUMP_QUEUE_TYPE_DEV, &dump)) rc = 5;
        if (vi.deinit() != maix::err::ERR_NONE) rc = 5;
        std::ofstream csv("frames.csv");
        csv << "sequence,pts_raw,monotonic_us,width,height,stride,frame_bytes,format\n";
        for (const auto &r : records)
            csv << r.seq << ',' << r.pts << ',' << r.mono << ',' << r.width << ',' << r.height
                << ',' << r.stride << ',' << r.size << ',' << r.format << '\n';
        std::ofstream health_csv("health.csv");
        health_csv << "elapsed_us,temperature_c,mipi_errors,rss_kb\n";
        for (const auto &h : health_records)
            health_csv << h.elapsed_us << ',' << h.temperature << ',' << h.mipi_errors << ',' << h.rss_kb << '\n';
        csv.flush(); health_csv.flush();
        if (!csv || !health_csv) rc=11;
        if (records.size() < 2 && !rc) rc = 5;
        const double fps = records.size() > 1 ?
            (records.size()-1) * 1000000.0 / (records.back().mono - records.front().mono) : 0;
        std::ofstream summary("capture.json");
        summary << "{\"requested_fps\":" << (hfr ? OS04A10_HFR_FPS : 30)
                << ",\"measurement\":\"" << (nv21 ? "VIN CH0 NV12 + asynchronous AX VENC" : "VIN IFE RAW") << "\",\"frames\":" << records.size()
                << ",\"monotonic_fps\":" << std::setprecision(10) << fps
                << ",\"acquire_errors\":" << acquire_errors << ",\"release_errors\":" << release_errors
                << ",\"recording\":true,\"encoded_submissions\":"<<encoder.submitted<<",\"encoded_packets\":"<<encoder.packets
                <<",\"encoder_queue_dropped\":"<<(sender ? worker_queue_rejected+sender->accepted-encoder.submitted : 0)
                <<",\"encoder_worker_rejected\":"<<worker_queue_rejected
                <<",\"encoder_send_errors\":"<<encoder.send_errors<<",\"encoder_release_errors\":"<<encoder.release_errors
                <<",\"encoder_queue_full_events\":"<<encoder.queue_full_events<<",\"encoder_retry\":"<<(venc_retry?"true":"false")
                <<",\"encoder_worker\":"<<(venc_worker?"true":"false")<<",\"encoder_worker_high_water\":"<<(sender?sender->high_water:0)
                <<",\"encoder_input_copy\":"<<(venc_copy?"true":"false")
                << ",\"encoder_fifo_depth\":" << venc_depth
                << ",\"queue_depth\":" << queue_depth
                << ",\"sequence_gap_events\":" << continuity.sequence_gap_events
                << ",\"sequence_missing\":" << continuity.sequence_missing
                << ",\"sequence_duplicates\":" << continuity.sequence_duplicates
                << ",\"sequence_backwards\":" << continuity.sequence_backwards
                << ",\"pts_nonincreasing\":" << continuity.pts_nonincreasing
                << ",\"continuity_ok\":" << (continuity.perfect() ? "true" : "false")
                << ",\"realtime\":" << (realtime ? "true" : "false")
                << ",\"temperature_max_c\":" << health_policy.temperature_max_c
                << ",\"temperature_limit_c\":" << health_policy.temperature_limit_c
                << ",\"mipi_errors_max\":" << health_policy.mipi_errors_max
                << ",\"mipi_error_limit\":" << health_policy.mipi_error_limit
                << ",\"recovered_mipi_error\":" << (health_policy.recovered_mipi_error() ? "true" : "false")
                << ",\"health_failure\":\"" << health_policy.failure_reason() << "\""
                << ",\"storage_low_or_query_failed\":" << (storage_low ? "true" : "false")
                << ",\"sample_saved\":" << (sampled ? "true" : "false")
                << ",\"exit_code\":" << rc << ",\"interrupted\":" << (stopped ? "true" : "false") << "}\n";
        std::cout << (nv21 ? "nv21_frames=" : "raw_frames=") << records.size() << " monotonic_fps=" << fps << " result=" << rc << '\n';
    } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 8; }
    return rc;
}
