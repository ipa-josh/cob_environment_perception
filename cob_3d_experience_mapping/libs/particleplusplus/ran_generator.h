#pragma once

#include <random>
#include <chrono>
namespace particle_plus_plus {

class RandomGenerator {
 public:
  unsigned seed;

  RandomGenerator() {
    seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator.seed(seed);
  }

  static RandomGenerator& getInstance() {
    static RandomGenerator A;
    return A;
  }

  std::default_random_engine& get_gen() { return generator; }

 private:
  std::default_random_engine generator;
};
}