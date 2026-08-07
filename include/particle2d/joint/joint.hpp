#pragma once

#include "particle2d/types.hpp"
#include "particle2d/vector.hpp"

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
  
  AngleGrad minAngle;
  AngleGrad maxAngle;

  float shiftness;
};
