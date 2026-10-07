#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace dart {
// Experimental bounded RGB component measurement. This is an observation,
// not a prediction or a complete target classifier; require separate NPU confirmation.
struct FastCenterMeasurement {
    bool valid=false;
    float x=0,y=0,diameter=0;
    int pixels=0,peak_green=0;
};
inline FastCenterMeasurement measure_fast_green(const uint8_t *rgb,int w,int h,
                                               float anchor_x,float anchor_y,float gate) {
    if(!rgb||w<=0||h<=0||w>256||h>256||!std::isfinite(anchor_x)||
       !std::isfinite(anchor_y)||!std::isfinite(gate)||gate<=0)
        throw std::invalid_argument("invalid bounded center measurement");
    // Estimate the bright local core before flood filling. Otherwise dim green
    // rails/walls connected to the lamp make the component reach the ROI edge.
    int peak_green=0,peak_excess=0;
    const float radius2=(gate+8)*(gate+8);
    for(int i=0;i<w*h;++i) {
        float dx=i%w-anchor_x,dy=i/w-anchor_y;if(dx*dx+dy*dy>radius2)continue;
        const auto *p=rgb+3*i;int e=int(p[1])-std::max(p[0],p[2]);
        if(e>=16 && 100*e>=6*(p[0]+p[1]+p[2])) {
            peak_green=std::max(peak_green,int(p[1]));peak_excess=std::max(peak_excess,e);
        }
    }
    const int green_floor=std::max(80,peak_green*65/100),excess_floor=std::max(16,peak_excess*35/100);
    std::vector<uint8_t> mask(static_cast<size_t>(w)*h);
    for(int i=0;i<w*h;++i) {
        const auto *p=rgb+3*i;
        const int excess=int(p[1])-std::max(p[0],p[2]);
        mask[i]=p[1]>=green_floor && excess>=excess_floor && 100*excess>=6*(p[0]+p[1]+p[2]);
    }
    std::vector<int> queue;queue.reserve(w*h);
    FastCenterMeasurement best;double best_cost=1e30;
    for(int seed=0;seed<w*h;++seed) {
        if(!mask[seed])continue;
        queue.clear();queue.push_back(seed);mask[seed]=0;
        int x0=w,y0=h,x1=0,y1=0,peak=0;double weight=0,xsum=0,ysum=0;
        for(size_t head=0;head<queue.size();++head) {
            int i=queue[head],x=i%w,y=i/w;const auto *p=rgb+3*i;
            const double a=int(p[1])-std::max(p[0],p[2]);weight+=a;xsum+=a*x;ysum+=a*y;
            x0=std::min(x0,x);x1=std::max(x1,x);y0=std::min(y0,y);y1=std::max(y1,y);peak=std::max(peak,int(p[1]));
            for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
                int xx=x+dx,yy=y+dy;if(xx<0||yy<0||xx>=w||yy>=h)continue;
                int j=yy*w+xx;if(mask[j]){mask[j]=0;queue.push_back(j);}
            }
        }
        const int bw=x1-x0+1,bh=y1-y0+1;
        if(x0==0||y0==0||x1==w-1||y1==h-1||peak<160||
           (queue.size()<2&&peak<220)||std::max(bw,bh)>4*std::min(bw,bh)||
           queue.size()*10<static_cast<size_t>(bw*bh))continue;
        const float x=xsum/weight,y=ysum/weight;
        const double d=std::hypot(x-anchor_x,y-anchor_y);
        if(d>gate)continue;
        const double cost=d-0.01*peak;
        if(cost<best_cost) {
            best_cost=cost;best={true,x,y,float(std::max(bw,bh)),int(queue.size()),peak};
        }
    }
    return best;
}
} // namespace dart
