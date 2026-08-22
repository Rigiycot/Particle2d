#pragma once

#include "particle2d/particle/shape.hpp"
#include "particle2d/vector.hpp"

namespace p2 {
  struct Collision 
  {
    float overlap;
    Vec2 normal;
  };

  class World;

  Collision collide(p2::World& world, const Shape& a, const Shape& b);

  bool intersects(p2::World& world, const Shape& a, const Shape& b);
}
