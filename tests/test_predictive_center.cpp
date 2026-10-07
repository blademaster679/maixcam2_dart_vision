#include "dart/predictive_center.hpp"
#include <stdexcept>
#include <iostream>
void require(bool b){if(!b)throw std::runtime_error("predictive center test failed");}
int main(){dart::PredictiveCenter f;dart::PredictiveCenterResult r;
 for(int i=0;i<20;++i)r=f.update(true,100+i,200,10,1000000+i*5556,1);
 require(r.valid&&!r.predicted);
 const uint64_t t=1000000+19*5556;
 r=f.update(false,0,0,0,t+5556,2);require(r.valid&&r.predicted&&r.age_ms>0&&r.x>=119);
 r=f.update(true,800,500,10,t+11112,2);require(r.valid&&r.predicted&&r.rejected&&r.x<150);
 r=f.update(false,0,0,0,t+61000,2);require(!r.valid);
 for(int i=0;i<3;++i)r=f.update(true,800,500,10,t+70000+i*5556,3);
 require(r.valid&&!r.predicted&&r.reacquired&&r.x==800);
 r=f.update(false,0,0,0,t+90000,3,true);require(!r.valid);
 r=f.update(false,0,0,0,t+95556,3);require(!r.valid);
 f.reset();for(int i=0;i<3;++i)r=f.update(true,100,200,10,2000000+i*5556,1);
 require(r.valid);r=f.update(true,100,200,10,2011112,1);require(!r.valid);
 std::cout<<"short dropout, far outlier, horizon, re-acquisition, negative identity and timestamps passed\n";
}
