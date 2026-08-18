#pragma once

#include "particle2d/vector.hpp" 
#include <cstdint>

namespace p2
{
  class World;

  struct Collision
  {
    Vec2 normal;
    float depth;
  };

  class Solver
  {
    public:
      void integrate(World& world, float dt);
      void constraints(World& world, float dt, uint16_t iterations);
      void collisions(World& world, float dt);
  };
}
