#pragma once 

#include "particle2d/particle/particle.hpp"
#include "particle2d/types.hpp"
#include "particle2d/vector.hpp"
#include <cstdint>
#include <particle2d/world/world.hpp>

using maskSize = uint8_t;

class Engine 
{
  public:
    Engine() = default;

    void step(float dt);

    World& world();

    const World& world() const;

    BodyID createRectangle(ShapeID rect, maskSize category, maskSize collides);
    BodyID createCircle(ShapeID circle, maskSize category, maskSize collides);
    BodyID createCapsule(ShapeID capsule, maskSize category, maskSize collides);

    JointID createJoint(ParticleID a, ParticleID b, float length);
    AngleJointID createAngle(ParticleID a, ParticleID b, ParticleID c, AngleGrad min, AngleGrad max);
    

  private:
    World m_world;
};
