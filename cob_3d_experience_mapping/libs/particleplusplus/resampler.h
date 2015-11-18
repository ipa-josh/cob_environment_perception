#pragma once

#include <iostream>
#include <vector>
#include <random>
#include "ran_generator.h"
namespace particle_plus_plus {

template <class StateType, class PrecisionType = double>
class Resampler {
 public:
  Resampler() = delete;
  explicit Resampler(const std::vector<PrecisionType> &wi,
                     const std::vector<StateType> &xi2,
                     std::vector<StateType>* xi1)
      : wi_(wi), xi2_(xi2), xi1_(xi1) {
  }
  virtual void operator()() const {
    std::discrete_distribution<int> gen(wi_.begin(), wi_.end());
    for (typename std::vector<StateType>::iterator it = xi1_->begin();
         it < xi1_->end(); it++) {
      int index = gen(RandomGenerator::getInstance().get_gen());
      *it = xi2_[index];
    }
  }
  virtual ~Resampler() = default;

 private:
  // weight of each particle
  const std::vector<PrecisionType> &wi_;  
  // particles
  const std::vector<StateType> &xi2_;     
  std::vector<StateType> *xi1_;
};
}