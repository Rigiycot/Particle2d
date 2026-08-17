#pragma once

#include "particle2d/vector.hpp"

struct Particle 
{
  Vec2 pos;
  Vec2 prevPos;

  float inverseMass;

  bool isActive = true;
  bool useGravity = true;

  Particle(Vec2 pos = Vec2{0, 0}, float inverseMass = 1)
    : pos(pos), inverseMass(inverseMass)
  {};
};


