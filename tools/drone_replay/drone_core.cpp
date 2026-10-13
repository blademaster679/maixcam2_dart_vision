// Host RGB adapter for the upstream NV21 proposal and green-center primitives.
// No armor, pose, model inference, simulator identity or actuator access.
#include "dart/nv21_pipeline.hpp"
#include "dart/fast_center_measurement.hpp"
#include "dart/center_kalman.hpp"
#include "dart/model_confirmation_gate.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <string>

namespace {
thread_local std::string error;
std::vector<uint8_t> nv21_bytes(const uint8_t*rgb,int width,int height){
    cv::Mat image(height,width,CV_8UC3,const_cast<uint8_t*>(rgb)),i420;
    cv::cvtColor(image,i420,cv::COLOR_RGB2YUV_I420);
    const int pixels=width*height;std::vector<uint8_t> data(pixels*3/2);
    std::memcpy(data.data(),i420.data,pixels);
    for(int i=0;i<pixels/4;++i){data[pixels+2*i]=i420.data[pixels+pixels/4+i];data[pixels+2*i+1]=i420.data[pixels+i];}
    return data;
}
struct Tracker {
    uint64_t last=0, positive=0, generation=0, maximum_gap;
    bool active=false,have_time=false;
    unsigned hits=0;
    float x=0,y=0,size=0;
    dart::CenterKalman filter;
    dart::ModelConfirmationGate gate;
    explicit Tracker(uint64_t period):maximum_gap(period*2),filter(period*2){}
    void clear(){active=false;hits=0;positive=0;filter.reset();++generation;}
};
}
extern "C" {
const char* dg_error(){return error.c_str();}
int dg_raw_proposals(const uint8_t*rgb,int width,int height,float*output){
    try{
        if(!rgb||!output||width<=0||height<=0||width%2||height%2)throw std::invalid_argument("invalid RGB dimensions");
        auto data=nv21_bytes(rgb,width,height);
        dart::Nv21View view{data.data(),data.data()+width*height,width,height,width,width};
        auto candidates=dart::nv21_green_proposals(view);
        for(size_t i=0;i<candidates.size();++i){output[i*2]=candidates[i].x;output[i*2+1]=candidates[i].y;}
        return candidates.size();
    }catch(const std::exception&e){error=e.what();return -1;}
}
// Candidate rows: x, y, diameter, pixel count. Contiguous uint8 RGB only.
int dg_proposals(const uint8_t* rgb,int width,int height,float* output,int capacity,float maximum_aspect) {
    try {
        if(!rgb||!output||capacity<=0||width<=0||height<=0||width%2||height%2)
            throw std::invalid_argument("RGB image must have positive even dimensions");
        cv::Mat image(height,width,CV_8UC3,const_cast<uint8_t*>(rgb));
        const int pixels=width*height;
        auto nv21=nv21_bytes(rgb,width,height);
        dart::Nv21View view{nv21.data(),nv21.data()+pixels,width,height,width,width};
        const auto proposals=dart::nv21_green_proposals(view);
        int count=0;
        for(const auto&p:proposals){
            auto roi=dart::source_roi(p.x,p.y,128,width,height);
            cv::Mat crop=image(cv::Rect(roi.x,roi.y,roi.width,roi.height)).clone();
            auto m=dart::measure_fast_green(crop.data,crop.cols,crop.rows,p.x-roi.x,p.y-roi.y,48,maximum_aspect);
            if(!m.valid)continue;
            m.x+=roi.x;m.y+=roi.y;
            bool duplicate=false;
            for(int i=0;i<count;++i)if(std::hypot(output[4*i]-m.x,output[4*i+1]-m.y)<std::max(4.0F,m.diameter))duplicate=true;
            if(duplicate)continue;
            output[count*4]=m.x;output[count*4+1]=m.y;output[count*4+2]=m.diameter;output[count*4+3]=m.pixels;
            if(++count==capacity)break;
        }
        return count;
    }catch(const std::exception&e){error=e.what();return -1;}
}
void* dg_create(uint64_t period_us){if(!period_us||period_us>1000000){error="invalid frame period";return nullptr;}return new Tracker(period_us);}
void dg_destroy(void* tracker){delete static_cast<Tracker*>(tracker);}
// Result: valid, model_positive, hits, generation, raw x/y/diameter,
// smooth valid/x/y, state(0 lost,1 acquiring,2 tracking), reason,
// selected candidate index, score. All output centers are current measurements.
int dg_update(void* pointer,uint64_t ts,const float* candidates,const float* confirmations,int count,float* out) {
    try {
        if(!pointer||!out||count<0||count>5||(count&&(!candidates||!confirmations)))throw std::invalid_argument("invalid tracker arguments");
        auto&t=*static_cast<Tracker*>(pointer);std::fill(out,out+14,0);out[12]=-1;
        if(t.have_time&&ts<=t.last){t.clear();out[11]=1;out[3]=t.generation;return 0;}
        if(t.have_time&&ts-t.last>t.maximum_gap)t.clear();
        t.last=ts;t.have_time=true;
        int selected=-1;
        if(t.active){
            float best=std::clamp(t.size,16.0F,48.0F);
            for(int i=0;i<count;++i){const float*c=candidates+i*4;float d=std::hypot(c[0]-t.x,c[1]-t.y);
                if(d<=best&&c[2]>=t.size*.5F&&c[2]<=t.size*2){selected=i;best=d;}}
            if(selected<0)t.clear();
        }
        if(selected<0&&count){selected=0;for(int i=0;i<count;++i)if(confirmations[i*2]>0){selected=i;break;}}
        if(selected<0){t.clear();out[11]=2;out[3]=t.generation;return 0;}
        const float*c=candidates+selected*4;
        if(!std::isfinite(c[0])||!std::isfinite(c[1])||!std::isfinite(c[2]))throw std::invalid_argument("nonfinite measurement");
        if(!t.active){t.active=true;++t.generation;t.hits=0;}
        bool positive=confirmations[selected*2]>0;
        bool rejected=t.gate.reject(t.generation,positive,t.hits>=2);
        out[12]=selected;out[1]=positive;out[13]=confirmations[selected*2+1];
        out[4]=c[0];out[5]=c[1];out[6]=c[2];
        if(rejected){t.clear();out[11]=3;out[3]=t.generation;return 0;}
        if(positive){++t.hits;t.positive=ts;}
        t.x=c[0];t.y=c[1];t.size=c[2];
        bool valid=t.hits>=2&&t.positive&&ts-t.positive<=t.maximum_gap;
        auto smooth=t.filter.update(valid,t.x,t.y,t.size,ts,t.generation);
        out[0]=valid;out[2]=t.hits;out[3]=t.generation;out[4]=t.x;out[5]=t.y;out[6]=t.size;
        out[7]=smooth.valid;out[8]=smooth.x;out[9]=smooth.y;out[10]=valid?2:1;out[11]=valid?0:4;
        return 0;
    }catch(const std::exception&e){error=e.what();return -1;}
}
}
