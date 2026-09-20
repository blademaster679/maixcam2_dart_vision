// Host regression tests for bounded queue-full recovery; no hardware/encoding emulation claims.
#include <cassert>
#include <deque>
#include <mutex>
#include <map>
#include "direct_venc.hpp"
#include "vin_venc_queue.hpp"
static std::mutex guard;
static std::deque<AX_VIDEO_FRAME_INFO_T> pending;
static AX_VENC_CHN_ATTR_T config;
static std::atomic<int> attempts{0};
static int mode=0;
static std::chrono::steady_clock::time_point retry_start;
static std::map<unsigned long long,unsigned> releases;
static std::vector<unsigned char> input_pixels(640*360*3/2),output_pixels(input_pixels.size());
static bool invalidated=false,flushed=false,fail_invalidate=false;
static int block_releases=0,pool_destroys=0;
extern "C" {
AX_S32 AX_VENC_Init(const AX_VENC_MOD_ATTR_T*) { return 0; }
AX_S32 AX_VENC_Deinit() { return 0; }
AX_S32 AX_VENC_CreateChn(VENC_CHN,const AX_VENC_CHN_ATTR_T *p) {config=*p;return 0;}
AX_S32 AX_VENC_GetChnAttr(VENC_CHN,AX_VENC_CHN_ATTR_T *p) {*p=config;return 0;}
AX_S32 AX_VENC_DestroyChn(VENC_CHN) {return 0;}
AX_S32 AX_VENC_StartRecvFrame(VENC_CHN,const AX_VENC_RECV_PIC_PARAM_T*) {return 0;}
AX_S32 AX_VENC_StopRecvFrame(VENC_CHN) {return 0;}
AX_S32 AX_VENC_SendFrame(VENC_CHN,const AX_VIDEO_FRAME_INFO_T *p,AX_S32) {
 int count=++attempts;
 if((mode==5 && std::chrono::steady_clock::now()-retry_start<std::chrono::milliseconds(40)) || mode==1 || (mode==0 && count<=2))return AX_ERR_VENC_QUEUE_FULL;
 if(mode==2)return AX_ERR_VENC_ILLEGAL_PARAM;
 std::lock_guard<std::mutex> lock(guard);pending.push_back(*p);return 0;
}
AX_S32 AX_VENC_GetStream(VENC_CHN,AX_VENC_STREAM_T *p,AX_S32) {
 if(mode!=4)std::this_thread::sleep_for(std::chrono::milliseconds(1));
 std::lock_guard<std::mutex> lock(guard);
 if(pending.empty())return AX_ERR_VENC_BUF_EMPTY;
 auto f=pending.front();pending.pop_front();static AX_U8 data[1]={0};
 p->stPack.pu8Addr=data;p->stPack.u32Len=1;p->stPack.u64PTS=f.stVFrame.u64PTS;p->stPack.u64SeqNum=f.stVFrame.u64SeqNum;return 0;
}
AX_S32 AX_VENC_ReleaseStream(VENC_CHN,const AX_VENC_STREAM_T*) {return 0;}
AX_S32 AX_VENC_QueryStatus(VENC_CHN,AX_VENC_CHN_STATUS_T *p) {std::lock_guard<std::mutex> lock(guard);p->u32LeftPics=pending.size();return 0;}
AX_S32 AX_VIN_ReleaseYuvFrame(AX_U8,AX_VIN_CHN_ID_E,const AX_IMG_INFO_T *p) {std::lock_guard<std::mutex> lock(guard);++releases[p->tFrameInfo.stVFrame.u64SeqNum];return 0;}
AX_POOL AX_POOL_CreatePool(AX_POOL_CONFIG_T *p) {assert(p->BlkCnt==16 && p->CacheMode==AX_POOL_CACHE_MODE_CACHED);return 1;}
AX_S32 AX_POOL_DestroyPool(AX_POOL) {++pool_destroys;return 0;}
AX_BLK AX_POOL_GetBlock(AX_POOL,AX_U64,const AX_S8*) {return 42;}
AX_VOID *AX_POOL_GetBlockVirAddr(AX_BLK) {return output_pixels.data();}
AX_U64 AX_POOL_Handle2PhysAddr(AX_BLK) {return 0x200000;}
AX_S32 AX_POOL_ReleaseBlock(AX_BLK) {++block_releases;return 0;}
AX_VOID *AX_SYS_MmapCache(AX_U64,AX_U32) {return input_pixels.data();}
AX_S32 AX_SYS_MinvalidateCache(AX_U64,AX_VOID*,AX_U32) {invalidated=true;return fail_invalidate ? -1 : 0;}
AX_S32 AX_SYS_MflushCache(AX_U64,AX_VOID*,AX_U32) {assert(invalidated);flushed=true;return 0;}
AX_S32 AX_SYS_Munmap(AX_VOID*,AX_U32) {return 0;}
}
int main() {
 AX_VIDEO_FRAME_INFO_T frame={};frame.stVFrame.u64SeqNum=123;frame.stVFrame.u64PTS=123000;
 {DirectVenc encoder(640,360,360,true);assert(encoder.send(frame)==0);encoder.finish();assert(attempts==3);assert(encoder.queue_full_events==2);assert(encoder.submitted==1 && encoder.packets==1 && !encoder.failed);}
 attempts=0;mode=1;
 {DirectVenc encoder(640,360,360,true);auto start=std::chrono::steady_clock::now();assert(encoder.send(frame)==AX_ERR_VENC_QUEUE_FULL);auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count();encoder.finish();assert(ms>=20 && ms<1000);assert(encoder.submitted==0 && encoder.send_errors==1);}
 attempts=0;mode=5;retry_start=std::chrono::steady_clock::now();
 {DirectVenc encoder(640,360,360,true,0,8,12000,200000);assert(encoder.send(frame)==0);encoder.finish();assert(encoder.queue_full_events>0 && encoder.submitted==1 && encoder.packets==1 && !encoder.failed);}
 attempts=0;mode=1;
 {DirectVenc encoder(640,360,360,true,0,8,12000,200000);auto start=std::chrono::steady_clock::now();assert(encoder.send(frame)==AX_ERR_VENC_QUEUE_FULL);auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count();encoder.finish();assert(ms>=200 && ms<1000 && encoder.submitted==0);}
 attempts=0;mode=2;
 {DirectVenc encoder(640,360,360,true);assert(encoder.send(frame)==AX_ERR_VENC_ILLEGAL_PARAM);encoder.finish();assert(attempts==1 && encoder.submitted==0);}
 for(int error_mode:{0,2}) {
  mode=error_mode;attempts=0;releases.clear();
  DirectVenc encoder(640,360,360,true);VinVencQueue sender(encoder);
  for(unsigned i=0;i<40;++i) {AX_IMG_INFO_T image={};image.tFrameInfo.stVFrame.u64SeqNum=i;
   if(!sender.push(image))AX_VIN_ReleaseYuvFrame(0,AX_VIN_CHN_ID_MAIN,&image);}
  sender.finish();encoder.finish();
  assert(sender.accepted==sender.released);assert(sender.high_water<=32);assert(releases.size()==40);
  for(const auto &r:releases)assert(r.second==1);
  if(error_mode==0)assert(encoder.submitted==sender.accepted && encoder.packets==sender.accepted && !sender.failed);
  else assert(sender.failed);
 }
 {
  VencInputCopy pool;AX_VIDEO_FRAME_INFO_T input={},output={};auto &f=input.stVFrame;
  f.u32Width=640;f.u32Height=360;f.u32PicStride[0]=640;f.u32FrameSize=input_pixels.size();
  f.enImgFormat=AX_FORMAT_YUV420_SEMIPLANAR;f.u64SeqNum=567;f.u64PTS=567000;f.u64PrivateData=999;
  for(size_t i=0;i<input_pixels.size();++i)input_pixels[i]=i%251;
  assert(pool.copy(input,output)==0 && invalidated && flushed);
  assert(input_pixels==output_pixels && output.stVFrame.u64SeqNum==567 && output.stVFrame.u64PTS==567000);
  assert(output.stVFrame.u64PrivateData==0 && input.stVFrame.u64PrivateData==999);
  AX_POOL_ReleaseBlock(output.stVFrame.u32BlkId[0]);fail_invalidate=true;
  assert(pool.copy(input,output)!=0 && block_releases==2);assert(pool.close()==0);
 }
 assert(pool_destroys==1);
 mode=3;attempts=0;
 {DirectVenc encoder(640,360,360,true,180,8);assert(config.stVencAttr.u8InFifoDepth==8 && config.stVencAttr.u8OutFifoDepth==8);assert(encoder.send(frame)==0);encoder.finish();assert(encoder.packets==1 && !encoder.failed);}
 mode=3;attempts=0;
 {DirectVenc encoder(640,360,180,true,0,8,12000,200000,1);
  assert(encoder.send(frame)==0);std::this_thread::sleep_for(std::chrono::milliseconds(5));
  assert(encoder.send(frame)==0);encoder.finish();assert(!encoder.failed);
  std::ifstream checkpoint("durable_checkpoint.json");std::string text((std::istreambuf_iterator<char>(checkpoint)),{});
  assert(text.find("\"frames\":2")!=std::string::npos);
 }
 mode=4;attempts=0;
 {DirectVenc encoder(1344,760,180,false,0,8,601*400);
  for(unsigned i=0;i<108100;++i) {frame.stVFrame.u64SeqNum=i;assert(encoder.send(frame)==0);}
  encoder.finish();assert(encoder.submitted==108100 && encoder.packets==108100 && !encoder.failed);
  std::ifstream log("venc_send.csv");std::string line;size_t lines=0;
  while(std::getline(log,line))++lines;
  assert(lines==108101);
 }
 std::cout<<"PASS: ten-minute frame volume retains all send metadata beyond previous 12000-frame limit\n";
 std::cout<<"PASS: transient full retries same frame once; persistent full bounded; other errors not retried\n";
 std::cout<<"PASS: worker owns accepted VIN frames only; releases all frames once after success, overload, or encoder failure\n";
 std::cout<<"PASS: user-pool copy preserves pixels and frame identity, performs DMA cache operations, and cleans up on error\n";
}
