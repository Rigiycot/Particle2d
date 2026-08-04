#pragma once

#include "particle/vector.hpp"

#include <cstdint>

using ParticleID = uint8_t;

struct Particle 
{
  Vec2 pos;
  Vec2 prevPos;

  float inverseMass;
};


