#include "dart/fast_center_measurement.hpp"
#include <iostream>
#include <vector>
#include <stdexcept>
int main(){
 try {
  const int w=128;std::vector<uint8_t> rgb(w*w*3,10);
  auto pixel=[&](int x,int y,int r,int g,int b){auto i=(y*w+x)*3;rgb[i]=r;rgb[i+1]=g;rgb[i+2]=b;};
  auto check=[](bool ok,const char*what){if(!ok)throw std::runtime_error(what);};
  check(!dart::measure_fast_green(rgb.data(),w,w,64,64,24).valid,"empty must not create observation");
  for(int y=0;y<w;++y)for(int x=0;x<w;++x)pixel(x,y,20,200,220);
  check(!dart::measure_fast_green(rgb.data(),w,w,64,64,24).valid,"cyan wall rejected");
  for(int y=0;y<w;++y)for(int x=0;x<w;++x)pixel(x,y,20,240,20);
  check(!dart::measure_fast_green(rgb.data(),w,w,64,64,24).valid,"border-connected green surface rejected");
  std::fill(rgb.begin(),rgb.end(),10);
  for(int y=54;y<=74;++y)for(int x=54;x<=74;++x){int d=(x-64)*(x-64)+(y-64)*(y-64);if(d<100)pixel(x,y,d<16?255:15,255,d<16?255:15);}
  auto m=dart::measure_fast_green(rgb.data(),w,w,65,62,24);
  check(m.valid&&std::abs(m.x-64)<.01&&std::abs(m.y-64)<.01,"white core green halo yields full center");
  for(int y=28;y<=36;++y)for(int x=28;x<=36;++x)pixel(x,y,0,255,0);
  m=dart::measure_fast_green(rgb.data(),w,w,64,64,24);
  check(m.valid&&std::abs(m.x-64)<.01,"spatial gate rejects distractor");
  check(!dart::measure_fast_green(rgb.data(),w,w,100,100,10).valid,"no prediction or reuse outside gate");
  for(int x=0;x<128;++x)pixel(x,70,15,130,15);
  m=dart::measure_fast_green(rgb.data(),w,w,64,64,24);
  check(m.valid&&std::abs(m.x-64)<1.0,"dim connected rail must not consume lamp component");
  bool threw=false;try{dart::measure_fast_green(nullptr,w,w,64,64,24);}catch(const std::invalid_argument&){threw=true;}check(threw,"invalid input rejected");
  std::cout<<"fast center measurement tests passed\n";return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
