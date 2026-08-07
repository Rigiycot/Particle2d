#pragma once

#include "particle2d/types.hpp"

struct Joint
{
  ParticleID a;
  ParticleID b;

  float length;
  float shiftness;
};

struct AngleJoint
{
  ParticleID a;
  ParticleID b;
  ParticleID c;
  
  float minAngle;
  float maxAngle;

  float shiftness;
};
