#pragma once

#include <functional>
namespace particle_plus_plus {

template <class StateType, class ObsvType, class PrecisionType = double>
class Proposal {
 public:
  Proposal() = delete;
  explicit Proposal(PrecisionType (*f)(StateType, StateType, ObsvType))
      : proposal_fn_(f) {
    // ctor
  }
  virtual ~Proposal() = default;
  PrecisionType virtual operator()(const StateType a, const StateType b,
                                   const ObsvType c) const {
    return proposal_fn_(a, b, c);
  }

 private:
  std::function<PrecisionType(StateType, StateType, ObsvType)> proposal_fn_;
};
}