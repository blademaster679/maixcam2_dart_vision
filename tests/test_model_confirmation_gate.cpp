#include "dart/model_confirmation_gate.hpp"
#include "dart/predictive_center.hpp"
#include <stdexcept>
void require(bool b){if(!b)throw std::runtime_error("confirmation gate test failed");}
int main(){dart::ModelConfirmationGate g;
 require(g.reject(1,false,false));
 require(!g.reject(2,false,true));require(g.reject(2,false,true));
 require(!g.reject(2,true,true));require(!g.reject(2,false,true));
 require(!g.reject(3,false,true));require(!g.reject(3,true,true));
 dart::PredictiveCenter f;dart::PredictiveCenterResult r;
 for(int i=0;i<3;++i)r=f.update(true,100,200,6,1000000+i*5556,10);
 require(r.valid);
 r=f.update(false,0,0,0,1020000,11,true,11);require(r.valid&&r.predicted);
 r=f.update(false,0,0,0,1025556,11,true,10);require(!r.valid);
}
