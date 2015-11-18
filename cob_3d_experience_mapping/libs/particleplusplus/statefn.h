#pragma once

#include <functional>
namespace particle_plus_plus {

template <class StateType, class PrecisionType = double>
class StateFn {
 public:
  explicit StateFn(PrecisionType (*f)(StateType, StateType)) : state_fn_(f) {}
  StateFn() = delete;
  virtual ~StateFn() = default;
  PrecisionType virtual operator()(const StateType a, const StateType b) const {
    return state_fn_(a, b);
  }

 private:
  std::function<PrecisionType(StateType, StateType)> state_fn_;
};
}