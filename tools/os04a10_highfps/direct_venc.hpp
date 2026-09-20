#pragma once
// Asynchronous AX VENC use follows the device-matched MSP sample/common/common_venc.c.
// SendFrame retains pool buffers: the sample releases its pool reference after a successful send.
#include <atomic>
#include <thread>
#include <stdexcept>
#include <fstream>
#include <chrono>
#include <vector>
#include <iostream>
#include "ax_venc_api.h"
#include "durable_checkpoint.hpp"
class DirectVenc {
    bool initialized=false,created=false,receiving=false,finished=false;
    std::atomic<bool> done{false};
    std::thread drain;
    bool retry_full=false;
    unsigned retry_budget_us=20000;
    unsigned checkpoint_interval_ms=0;
    struct SendLog { unsigned long long seq,start,elapsed; unsigned retries; int result,status_result; unsigned left_pics,left_streams; };
    std::vector<SendLog> send_log;
    static unsigned long long clock_us() { return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
    static void require(int code,const char *what) {
        if(code) throw std::runtime_error(std::string(what)+" returned "+std::to_string(code));
    }
public:
    std::atomic<bool> failed{false};
    std::atomic<unsigned long> submitted{0},packets{0},send_errors{0},release_errors{0};
    unsigned long queue_full_events=0;
    DirectVenc(unsigned w,unsigned h,unsigned fps,bool retry=false,unsigned rc_fps=0,unsigned fifo_depth=4,size_t log_capacity=12000,unsigned retry_budget=20000,unsigned sync_interval_ms=0):retry_full(retry),retry_budget_us(retry_budget),checkpoint_interval_ms(sync_interval_ms) {
        try {
            send_log.reserve(log_capacity);
            AX_VENC_MOD_ATTR_T mod={};mod.enVencType=AX_VENC_VIDEO_ENCODER;
            mod.stModThdAttr.u32TotalThreadNum=2;
            require(AX_VENC_Init(&mod),"VENC_Init");initialized=true;
            AX_VENC_CHN_ATTR_T a={};
            a.stVencAttr.enType=PT_H264;
            a.stVencAttr.u32PicWidthSrc=a.stVencAttr.u32MaxPicWidth=w;
            a.stVencAttr.u32PicHeightSrc=a.stVencAttr.u32MaxPicHeight=h;
            a.stVencAttr.enLinkMode=AX_UNLINK_MODE;a.stVencAttr.enMemSource=AX_MEMORY_SOURCE_CMM;
            a.stVencAttr.u8InFifoDepth=fifo_depth;a.stVencAttr.u8OutFifoDepth=fifo_depth;
            a.stVencAttr.enProfile=AX_VENC_H264_MAIN_PROFILE;a.stVencAttr.enLevel=AX_VENC_H264_LEVEL_5_1;
            a.stVencAttr.u32BufSize=4*1024*1024;
            // RC metadata cap matches Sipeed's example. Acquisition/encoded sequence is checked separately.
            a.stRcAttr.stFrameRate.fSrcFrameRate=a.stRcAttr.stFrameRate.fDstFrameRate=rc_fps ? rc_fps : std::min(fps,180u);
            a.stRcAttr.enRcMode=AX_VENC_RC_MODE_H264CBR;a.stRcAttr.s32FirstFrameStartQp=-1;
            auto &c=a.stRcAttr.stH264Cbr;
            c.u32Gop=fps;c.u32BitRate=w>640?24000:12000;
            c.u32MinQp=c.u32MinIQp=18;c.u32MaxQp=c.u32MaxIQp=42;
            c.s32IntraQpDelta=-2;c.u32IdrQpDeltaRange=10;c.s32DeBreathQpDelta=-2;
            c.u32MinIprop=10;c.u32MaxIprop=40;
            a.stGopAttr.enGopMode=AX_VENC_GOPMODE_NORMALP;
            require(AX_VENC_CreateChn(0,&a),"VENC_CreateChn");created=true;
            AX_VENC_CHN_ATTR_T actual={};require(AX_VENC_GetChnAttr(0,&actual),"VENC_GetChnAttr");
            std::ofstream config("venc_config.json");config<<"{\"rc_src_fps\":"<<actual.stRcAttr.stFrameRate.fSrcFrameRate
                <<",\"rc_dst_fps\":"<<actual.stRcAttr.stFrameRate.fDstFrameRate<<",\"input_depth\":"<<unsigned(actual.stVencAttr.u8InFifoDepth)
                <<",\"output_depth\":"<<unsigned(actual.stVencAttr.u8OutFifoDepth)<<",\"retry_queue_full\":"<<(retry_full?"true":"false")<<"}\n";
            AX_VENC_RECV_PIC_PARAM_T recv={};recv.s32RecvPicNum=-1;
            require(AX_VENC_StartRecvFrame(0,&recv),"VENC_StartRecvFrame");receiving=true;
            drain=std::thread([this] {
                try {
                    std::ofstream stream("record.h264",std::ios::binary);
                    std::ofstream index("encoded_frames.csv");index<<"index,sequence,pts_raw,monotonic_us,bytes\n";
                    if(!stream||!index){failed=true;return;}
                    DurableCheckpoint durable;
                    const auto checkpoint_start=clock_us();
                    auto last_checkpoint=checkpoint_start;
                    auto snapshot=[&] {
                        stream.flush();index.flush();
                        if(!stream||!index)throw std::runtime_error("checkpoint stream flush failed");
                        auto vb=stream.tellp(),ib=index.tellp();
                        if(vb<0||ib<0)throw std::runtime_error("checkpoint offset failed");
                        return DurableCheckpoint::Snapshot{static_cast<unsigned long long>(vb),static_cast<unsigned long long>(ib),packets.load(),(clock_us()-checkpoint_start)/1000};
                    };
                    unsigned drain_timeouts=0;
                    while(!done || packets<submitted) {
                        AX_VENC_STREAM_T packet={};int rc=AX_VENC_GetStream(0,&packet,retry_full ? 0 : 200);
                        if(rc) {
                            if(done && ++drain_timeouts>=(retry_full ? 20000u : 25u)) {failed=true;break;}
                            if(retry_full)std::this_thread::sleep_for(std::chrono::microseconds(250));
                            continue;
                        }
                        drain_timeouts=0;
                        const auto &p=packet.stPack;
                        stream.write(reinterpret_cast<const char *>(p.pu8Addr),p.u32Len);
                        const auto now=std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
                        index<<packets<<','<<p.u64SeqNum<<','<<p.u64PTS<<','<<now<<','<<p.u32Len<<'\n';
                        ++packets;
                        if(AX_VENC_ReleaseStream(0,&packet)) {++release_errors;failed=true;break;}
                        if(!stream||!index) {failed=true;break;}
                        if(checkpoint_interval_ms && durable.ready() && clock_us()-last_checkpoint>=checkpoint_interval_ms*1000ULL) {
                            durable.request(snapshot());last_checkpoint=clock_us();
                        }
                    }
                    stream.flush();index.flush();if(!stream||!index)failed=true;
                    if(checkpoint_interval_ms) {durable.wait();DurableCheckpoint::sync(snapshot());}
                }catch(const std::exception &e){std::cerr<<"VENC drain/checkpoint: "<<e.what()<<'\n';failed=true;}catch(...){failed=true;}
            });
        }catch(...){finish();throw;}
    }
    int send(const AX_VIDEO_FRAME_INFO_T &frame) {
        const auto start=clock_us();unsigned retries=0;int rc;
        do {
            rc=AX_VENC_SendFrame(0,&frame,retry_full ? 0 : 20);
            if(rc==AX_ERR_VENC_QUEUE_FULL) ++queue_full_events;
            if(rc!=AX_ERR_VENC_QUEUE_FULL || !retry_full || clock_us()-start>=retry_budget_us) break;
            ++retries;std::this_thread::sleep_for(std::chrono::microseconds(250));
        }while(!failed);
        AX_VENC_CHN_STATUS_T status={};int status_rc=-1;
        if(rc || retries) status_rc=AX_VENC_QueryStatus(0,&status);
        if(send_log.size()<send_log.capacity())send_log.push_back({frame.stVFrame.u64SeqNum,start,clock_us()-start,retries,rc,status_rc,status.u32LeftPics,status.u32LeftStreamFrames});
        else {failed=true;return AX_ERR_VENC_NOMEM;}
        if(rc)std::cerr<<"VENC send status="<<(rc==AX_ERR_VENC_QUEUE_FULL?"QUEUE_FULL":rc==AX_ERR_VENC_TIMEOUT?"TIMEOUT":"OTHER")<<" code=0x"<<std::hex<<static_cast<unsigned>(rc)<<std::dec<<'\n';
        if(rc)++send_errors;else ++submitted;
        return rc;
    }
    void finish() {
        if(finished)return;
        finished=true;done=true;
        if(drain.joinable())drain.join();
        std::ofstream log("venc_send.csv");log<<"sequence,start_us,elapsed_us,retries,result,status_result,left_pics,left_stream_frames\n";
        for(const auto &r:send_log)log<<r.seq<<','<<r.start<<','<<r.elapsed<<','<<r.retries<<','<<r.result<<','<<r.status_result<<','<<r.left_pics<<','<<r.left_streams<<'\n';
        log.flush();if(!log)failed=true;
        if(receiving && AX_VENC_StopRecvFrame(0))failed=true;
        if(created && AX_VENC_DestroyChn(0))failed=true;
        if(initialized && AX_VENC_Deinit())failed=true;
    }
    ~DirectVenc(){finish();}
};
