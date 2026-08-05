#pragma once

#include "types.hpp"

struct Joint
{
  ParticleID a;
  ParticleID b;

  float length;
};

struct AngleJoint
{
  ParticleID a;
  ParticleID b;
  ParticleID c;
  
  float minAngle;
  float maxAngle;
};
