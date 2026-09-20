#include "durable_checkpoint.hpp"
#include <cassert>
#include <fstream>
#include <iterator>
#include <cstdio>
int main(){
 {std::ofstream v("record.h264");v<<"video";std::ofstream i("encoded_frames.csv");i<<"index";}
 DurableCheckpoint c;
 assert(c.request({5,5,1,30000}));c.wait();
 std::ifstream file("durable_checkpoint.json");std::string text((std::istreambuf_iterator<char>(file)),{});
 assert(text.find("\"frames\":1")!=std::string::npos);
 assert(text.find("\"video_bytes\":5")!=std::string::npos);
 {std::ofstream v("record.h264",std::ios::app);v<<"more";}
 assert(c.request({9,5,2,60000}));c.wait();
 std::ifstream next("durable_checkpoint.json");text.assign(std::istreambuf_iterator<char>(next),{});
 assert(text.find("\"video_bytes\":9")!=std::string::npos);
 assert(::unlink("encoded_frames.csv")==0);
 assert(c.request({9,5,2,90000}));bool failed=false;
 try{c.wait();}catch(const std::exception&){failed=true;}
 assert(failed); // failed sync must not publish a new successful checkpoint
 std::ifstream old("durable_checkpoint.json");text.assign(std::istreambuf_iterator<char>(old),{});
 assert(text.find("90000")==std::string::npos);
}
