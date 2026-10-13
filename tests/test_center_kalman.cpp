#include "dart/center_kalman.hpp"
#include <stdexcept>
#include <limits>
#include <iostream>
void require(bool b){if(!b)throw std::runtime_error("Kalman test failed");}
int main(){dart::CenterKalman f;
 double raw=0,smoothed=0;
 for(int i=0;i<600;++i){double noise=i%2?2:-2;auto r=f.update(true,100+noise,200-noise,10,1000000+i*5556,1);require(r.valid);if(i>100){raw+=noise*noise;smoothed+=(r.x-100)*(r.x-100);}}
 require(smoothed<raw*.25);
 require(!f.update(false,100,200,10,5000000,1).valid);
 auto r=f.update(true,400,500,10,5005556,1);require(r.reset&&r.x==400&&r.y==500);
 r=f.update(true,700,300,10,5011112,2);require(r.reset&&r.x==700);
 r=f.update(true,701,300,10,5011112,2);require(r.reset&&r.x==701);
 r=f.update(true,702,300,10,5111112,2);require(r.reset&&r.x==702);
 require(!f.update(true,std::numeric_limits<double>::quiet_NaN(),0,10,5200000,2).valid);
 f.reset();double max_error=0;
 for(int i=0;i<600;++i){double x=100+i*5556e-6*300;auto a=f.update(true,x,100,10,1000000+i*5556,1);if(i>100)max_error=std::max(max_error,std::abs(a.x-x));}
 require(max_error<.1); // constant-velocity tracking should not retain an EMA-like lag
 // Bounded response to a small step, with no future samples or buffering.
 f.reset();int response=-1;
 for(int i=0;i<400;++i){auto a=f.update(true,i<300?100:103,100,10,1000000+i*5556,1);if(i>=300&&response<0&&a.x>=102.7)response=i-300;}
 require(response>=0&&response*5.556<30);
 std::cout<<"step t90_ms="<<response*5.556<<" stationary noise ratio="<<smoothed/raw<<" ramp max error="<<max_error<<'\n';
}
