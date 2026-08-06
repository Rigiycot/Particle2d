#pragma once

#include "particle2d/vector.hpp"

struct Particle 
{
  Vec2 pos;
  Vec2 prevPos;

  float inverseMass;
};


