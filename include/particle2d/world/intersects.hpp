#pragma once

#include "particle2d/particle/shape.hpp"
#include "particle2d/vector.hpp"

namespace p2 {
  struct Collision 
  {
    float overlap;
    Vec2 normal;
  };

  Collision collide(const Shape& a, const Shape& b);

  bool intersects(const Shape& a, const Shape& b);
}
