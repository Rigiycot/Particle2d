#pragma once

#include "particle/particle.hpp"

#include <cstdint>

using JointID = uint8_t;

struct Joint
{

  ParticleID a;
  ParticleID b;

  float length;

  enum Type 
  {
    Dynamic,
    Static,
    Spring
  };

  Type jointType;
};
