#pragma once

#include <vector>
#include <functional>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <random>

#include "statefn.h"
#include "observefn.h"
#include "proposal.h"
#include "sampler.h"
#include "resampler.h"

/// this is the particle filter template
/// To use it, the user should define the state type and observation type
/// where the operator '+', '<<' and '>>' of state type should be predefined.
/// To construct an object, the user should give four function pointers as
/// listed below.
/// The user can also use four function objects to initialize the Pfilter
/// object.
/// In that case, automatic type conversion wi_ll be called.

namespace particle_plus_plus {

template <class StateType, class ObsvType, class PrecisionType = double>
class Pfilter {
 public:
  /// hide these functions
  Pfilter() = delete;
  Pfilter(const Pfilter &other) = delete;
  Pfilter &operator=(const Pfilter &other) = delete;
  virtual ~Pfilter() = default;

  Pfilter(PrecisionType (*fptr)(StateType, StateType),
          PrecisionType (*gptr)(StateType, ObsvType),
          PrecisionType (*qptr)(StateType, StateType, ObsvType),
          StateType (*q_sam_ptr)(StateType, ObsvType))
      : state_fn_(fptr),
        observe_fn_(gptr),
        proposal_fn_(qptr),
        sampler_(q_sam_ptr),
        resampler_(wi_, xi2_, &xi1_),
        particlenum_(0) {}  ///< declaration of constructor

  StateType iterate(ObsvType observe_input) {
    transform(xi1_.begin(), xi1_.end(), xi2_.begin(),
              std::bind(sampler_, std::placeholders::_1, observe_input));
    transform(xi2_.begin(), xi2_.end(), xi1_.begin(), wi_.begin(),
              std::bind(&Pfilter::ComposeFn, this, std::placeholders::_1,
                        std::placeholders::_2, observe_input));
    resampler_();
    return accumulate(xi1_.begin(), xi1_.end(), 0.0) / particlenum_;
  }
  void initialize(int pn) {
    particlenum_ = pn;
    xi1_.assign(particlenum_, 0);
    xi2_.assign(particlenum_, 0);
    wi_.assign(particlenum_, 0);
  }
  ///< initialize wi_th the number of particles we want to use

 private:
  PrecisionType ComposeFn(const StateType a, const StateType b,
                          const ObsvType c) const {
    return state_fn_(a, b) * observe_fn_(a, c) / proposal_fn_(a, b, c);
  }

  std::vector<StateType> xi1_;     ///< particles
  std::vector<StateType> xi2_;     ///< particles
  std::vector<PrecisionType> wi_;  ///< weights of particles

  StateFn<StateType> state_fn_;                ///< pdf for state move
  ObserveFn<StateType, ObsvType> observe_fn_;  ///< pdf for observation function
  Proposal<StateType, ObsvType>
      proposal_fn_;  ///< pdf of the proposal distribution
  Sampler<StateType, ObsvType>
      sampler_;                     ///< sampler of the proposal distribution
  Resampler<StateType> resampler_;  ///< resampler

  int particlenum_;  ///< number of particles
};
}
