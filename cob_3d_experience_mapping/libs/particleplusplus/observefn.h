#pragma once

#include <functional>
namespace particle_plus_plus {

template <class StateType, class ObsvType, class PrecisionType=double>
class ObserveFn {
 public:
  ObserveFn() = delete;
  ObserveFn(PrecisionType (*f)(StateType, ObsvType)) : proposal_(f) {}
  virtual ~ObserveFn() = default;
  PrecisionType virtual operator()(const StateType a, const ObsvType b) const {
    return proposal_(a, b);
  }

 private:
  std::function<PrecisionType(StateType, ObsvType)> proposal_;
};
}