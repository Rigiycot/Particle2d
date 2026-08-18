#pragma once

#include <cstdint>

namespace p2
{
  class World;

  class Solver
  {
    public:
      void integrate(World& world, float dt);
      void constraints(World& world, float dt, uint16_t iterations);
      void collisions(World& world, float dt);
  };
}
