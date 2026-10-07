// Same output-only filter used in the board prototype, applied to saved raw measurements.
#include "dart/center_kalman.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
std::vector<std::string> split(const std::string &line){std::vector<std::string> v;std::stringstream s(line);std::string x;while(std::getline(s,x,','))v.push_back(x);return v;}
int main(int argc,char **argv){try{
 if(argc!=3||std::filesystem::exists(argv[2]))throw std::runtime_error("usage: filter_center_csv INPUT.csv NEW_OUTPUT.csv");
 std::ifstream in(argv[1]);std::ofstream out(argv[2]);std::string line;if(!std::getline(in,line))throw std::runtime_error("empty input");
 auto header=split(line);std::map<std::string,size_t> cols;for(size_t i=0;i<header.size();++i)cols[header[i]]=i;
 out<<line<<",smooth_valid,smooth_reset,smooth_x,smooth_y,filter_us\n";dart::CenterKalman filter;
 size_t count=0;while(std::getline(in,line)){
  auto v=split(line);auto get=[&](const char*k){return std::stod(v.at(cols.at(k)));};
  auto t=std::chrono::steady_clock::now();
  auto result=filter.update(get("observed")!=0,get("x"),get("y"),get("size"),1000000+std::llround(get("sequence")*1000000/180),static_cast<uint64_t>(get("generation")));
  const auto us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-t).count();
  out<<line<<','<<result.valid<<','<<result.reset<<','<<std::fixed<<std::setprecision(9)<<result.x<<','<<result.y<<','<<us<<'\n';++count;
 }
 out.close();if(!in.eof()||!out)throw std::runtime_error("IO failure");std::cout<<"filtered="<<count<<'\n';
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
