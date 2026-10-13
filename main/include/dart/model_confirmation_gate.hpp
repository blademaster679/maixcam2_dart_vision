#pragma once
#include <cstdint>
namespace dart {
// Accepted results only. An unconfirmed candidate still fails immediately;
// an established track needs two consecutive negative model results.
class ModelConfirmationGate {
 uint64_t generation_=0;unsigned negatives_=0;
 public:
 bool reject(uint64_t generation,bool positive,bool established) {
  if(generation!=generation_){generation_=generation;negatives_=0;}
  if(positive){negatives_=0;return false;}
  ++negatives_;return !established||negatives_>=2;
 }
};
}
