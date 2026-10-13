#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace dart {
struct SmoothedCenter { bool valid=false,reset=false; double x=0,y=0; };
// Causal constant-velocity Kalman filter on accepted image measurements only.
// No predicted output on missing measurements. Units: pixels and seconds.
class CenterKalman {
 struct Axis {double p=0,v=0,p00=0,p01=0,p11=0;};
 Axis x_,y_;bool initialized_=false;uint64_t timestamp_=0,generation_=0;
 static void predict(Axis &a,double dt) {
  constexpr double q=250000; // continuous white acceleration spectral density
  a.p+=dt*a.v;
  a.p00+=2*dt*a.p01+dt*dt*a.p11+q*dt*dt*dt/3;
  a.p01+=dt*a.p11+q*dt*dt/2;a.p11+=q*dt;
 }
 static void correct(Axis &a,double z,double r,bool maneuver) {
  const double e=z-a.p;
  // Follow a sustained maneuver quickly instead of increasing smoothing lag.
  if(maneuver)a.p00+=e*e*.25;
  const double p00=a.p00,p01=a.p01,p11=a.p11;
  const double k0=p00/(p00+r),k1=p01/(p00+r);
  a.p+=k0*e;a.v+=k1*e;
  // Joseph form avoids loss of positive covariance through cancellation.
  a.p00=(1-k0)*(1-k0)*p00+k0*k0*r;
  a.p01=(1-k0)*(p01-k1*p00)+k0*k1*r;
  a.p11=p11-2*k1*p01+k1*k1*(p00+r);
 }
 public:
 void reset(){initialized_=false;}
 SmoothedCenter update(bool valid,double x,double y,double diameter,uint64_t ts,uint64_t generation) {
  if(!valid||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(diameter)) {reset();return {};}
  const double sigma=std::clamp(diameter*.06,1.5,4.0),r=sigma*sigma;
  if(!initialized_||generation!=generation_||ts<=timestamp_||ts-timestamp_>50000) {
   x_={x,0,r,0,1000000};y_={y,0,r,0,1000000};initialized_=true;timestamp_=ts;generation_=generation;
   return {true,true,x,y};
  }
  const double dt=(ts-timestamp_)/1000000.0;timestamp_=ts;
  predict(x_,dt);predict(y_,dt);
  const double ex=x-x_.p,ey=y-y_.p;
  // A gross discontinuity should not be turned into a long trail.
  if(std::hypot(ex,ey)>std::max(40.0,diameter*4)) {reset();return update(valid,x,y,diameter,ts,generation);}
  const bool maneuver=ex*ex/(x_.p00+r)+ey*ey/(y_.p00+r)>16;
  correct(x_,x,r,maneuver);correct(y_,y,r,maneuver);
  return {true,false,x_.p,y_.p};
 }
};
}
