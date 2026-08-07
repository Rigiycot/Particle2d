#include "particle2d/world/world.hpp"
#include "particle2d/joint/joint.hpp"
#include "particle2d/particle/particle.hpp"
#include "particle2d/types.hpp"

#include <vector>


void World::step(float dt, uint8_t iterations)
{
  
}

void World::addVelocity(ParticleID id, const Vec2& velocity, float dt)
{
  Particle& prt = this->getParticle(id);

  prt.prevPos -= velocity * dt;
}

void World::dampVelocity(ParticleID id, float amount)
{
  Particle& prt = this->getParticle(id);

  Vec2 velocity = prt.pos - prt.prevPos;
  velocity *= amount;

  prt.prevPos = prt.pos - velocity;
}


Particle& World::getParticle(ParticleID id)
{
  return particles.at(particleIdxToVec.at(id));
}
Joint& World::getJoint(JointID id)
{
  return joints.at(jointIdxToVec.at(id));
}

AngleJoint& World::getAngleJoint(AngleJointID id)
{
  return anglejoints.at(anglejointIdxToVec.at(id));
}

Shape& World::getShape(ShapeID id)
{
  return *shapes.at(shapeIdxToVec.at(id));
}


ParticleID World::createParticle(const Particle& prt)
{
  ParticleID id;

  if (!this->particleFreeID.empty())
  {
    id = particleFreeID.back();
    particleFreeID.pop_back();
  }
  else 
  {
    id = nextParticleFreeID++;
  }

  particleIdxToVec[id] = particles.size();
  particles.push_back(prt);

  return id;
}

JointID World::createJoint(const Joint& jnt)
{
  JointID id;

  if (!this->jointFreeID.empty())
  {
    id = jointFreeID.back();
    jointFreeID.pop_back();
  }
  else 
  {
    id = nextJointFreeID++;
  }

  jointIdxToVec[id] = joints.size();
  joints.push_back(jnt);

  return id;
}

ShapeID World::createShape(std::unique_ptr<Shape>& shp)
{
  BodyID id;

  if (!this->shapeFreeID.empty())
  {
    id = shapeFreeID.back();
    shapeFreeID.pop_back();
  }
  else 
  {
    id = nextShapeFreeID++;
  }

  shapeIdxToVec[id] = shapes.size();
  shapes.push_back(std::move(shp));

  return id;
}
