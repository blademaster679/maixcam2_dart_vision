#pragma once
#include "dart/center_kalman.hpp"

namespace dart {
struct PredictiveCenterResult {
 bool valid=false,predicted=false,rejected=false,reacquired=false;
 double x=0,y=0,age_ms=0;
};
// Output-only bounded coast for a mostly horizontal target. This does not
// increase the count of image observations or confirm target identity.
class PredictiveCenter {
 CenterKalman filter_;
 bool live_=false;uint64_t last_ts_=0,last_observed_=0,generation_=0;
 double x_=0,y_=0,vx_=0,vy_=0,size_=10;
 unsigned pending_=0;double pending_x_=0,pending_y_=0;uint64_t pending_ts_=0;
 static constexpr uint64_t horizon_us=60000;
 bool confirm(double x,double y,uint64_t ts) {
  if(!pending_||ts<=pending_ts_||ts-pending_ts_>30000||std::hypot(x-pending_x_,y-pending_y_)>12)pending_=0;
  pending_x_=x;pending_y_=y;pending_ts_=ts;return ++pending_>=3;
 }
 public:
 void reset(){live_=false;pending_=0;vx_=vy_=0;filter_.reset();}
 PredictiveCenterResult update(bool observed,double x,double y,double size,uint64_t ts,uint64_t generation,bool identity_rejected=false,uint64_t rejected_generation=0) {
  if((identity_rejected&&(rejected_generation==0||rejected_generation==generation_))||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(size)|| (last_ts_&&ts<=last_ts_)) {reset();last_ts_=ts;return {};}
  last_ts_=ts;
  if(live_&&ts-last_observed_>horizon_us){reset();return {};}
  if(!live_) {
   if(!observed){pending_=0;return {};}
   if(!confirm(x,y,ts))return {};
   filter_.reset();auto s=filter_.update(true,x,y,size,ts,generation);
   live_=true;pending_=0;x_=s.x;y_=s.y;vx_=vy_=0;size_=size;last_observed_=ts;generation_=generation;
   return {true,false,false,true,x_,y_,0};
  }
  double age=(ts-last_observed_)/1000000.0;
  // Velocity decays after 20 ms; no extrapolation beyond the 60 ms horizon.
  const double travel=age<=.02?age:.02+.02*(1-std::exp(-(age-.02)/.02));
  double px=x_+vx_*travel,py=y_+vy_*travel*.5;
  bool rejected=false;
  if(observed) {
   const double gx=std::clamp(size_*1.0,32.0,80.0)+std::abs(vx_)*age*.25;
   const double gy=std::clamp(size_*.6,24.0,48.0)+std::abs(vy_)*age*.15;
   const double innovation=std::pow((x-px)/gx,2)+std::pow((y-py)/gy,2);
   // A new detector generation also needs continuity confirmation.
   const bool continuous=innovation<=1 && (generation==generation_||confirm(x,y,ts));
   if(continuous) {
    pending_=0;auto s=filter_.update(true,x,y,size,ts,generation);
    if(age>0){vx_=.65*vx_+.35*std::clamp((s.x-x_)/age,-1000.0,1000.0);vy_=.8*vy_+.2*std::clamp((s.y-y_)/age,-300.0,300.0);}
    if(std::abs(vx_)<5)vx_=0;if(std::abs(vy_)<5)vy_=0;
    x_=s.x;y_=s.y;size_=size;last_observed_=ts;generation_=generation;
    return {true,false,false,false,x_,y_,0};
   }
   rejected=true;
   if(innovation>1)pending_=0;
  }
  if(px<0||px>=1344||py<0||py>=760){reset();return {};}
  return {true,true,rejected,false,px,py,age*1000};
 }
};
}
