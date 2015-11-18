#pragma once

#include <functional>
namespace particle_plus_plus {

template <class StateType, class ObsvType>
class Sampler {
 public:
  Sampler() = delete;
  Sampler(StateType (*f)(StateType, ObsvType)) : sampler_(f){};
  virtual ~Sampler() = default;
  // Sampler(const Sampler& other);
  StateType virtual operator()(const StateType a, const ObsvType b) const {
    return sampler_(a, b);
  }

 private:
  std::function<StateType(StateType, ObsvType)> sampler_;
};
}