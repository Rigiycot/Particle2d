#include "particle2d/world/world.hpp"
#include "particle2d/joint/joint.hpp"
#include "particle2d/particle/particle.hpp"
#include "particle2d/types.hpp"

#include <vector>


void World::step(float dt, uint8_t iterations)
{
  this->solver.integrate(*this, dt);

  for (int i = 0; i < iterations; ++i)
  {
    solver.constraints(*this, dt, iterations);
    solver.collisions(*this, dt);
  }
}

void World::addVelocity(ParticleID id, const Vec2& velocity, float dt)
{
  Particle& prt = getParticle(id);

  prt.prevPos -= velocity * dt;
}

void World::dampVelocity(ParticleID id, float amount)
{
  Particle& prt = getParticle(id);

  Vec2 velocity = prt.pos - prt.prevPos;
  velocity *= amount;

  prt.prevPos = prt.pos - velocity;
}


Particle& World::getParticle(ParticleID id)
{
  return particles.at(particleIDToVec.at(id));
}

Joint& World::getJoint(JointID id)
{
  return joints.at(jointIDToVec.at(id));
}

AngleJoint& World::getAngleJoint(AngleJointID id)
{
  return anglejoints.at(anglejointIDToVec.at(id));
}

Shape& World::getShape(ShapeID id)
{
  return *shapes.at(shapeIDToVec.at(id));
}

Body& World::getBody(BodyID id)
{
  return bodies.at(shapeIDToVec.at(id));
}


ParticleID World::createParticle(const Particle& prt)
{
  ParticleID id;

  if (!particleFreeID.empty())
  {
    id = particleFreeID.back();
    particleFreeID.pop_back();
  }
  else 
  {
    id = nextParticleFreeID++;
  }

  particleIDToVec[id] = particles.size();
  particles.push_back(prt);

  return id;
}

JointID World::createJoint(const Joint& jnt)
{
  JointID id;

  if (!jointFreeID.empty())
  {
    id = jointFreeID.back();
    jointFreeID.pop_back();
  }
  else 
  {
    id = nextJointFreeID++;
  }

  jointIDToVec[id] = joints.size();
  joints.push_back(jnt);

  return id;
}

AngleJointID World::createAngleJoint(const AngleJoint& ajnt)
{
  AngleJointID id;

  if (!anglejointFreeID.empty())
  {
    id = anglejointFreeID.back();
    anglejointFreeID.pop_back();
  }
  else
  {
    id = nextJointFreeID++;
  }
  anglejointIDToVec[id] = anglejoints.size();
  anglejoints.push_back(ajnt);

  return id;
}

ShapeID World::createShape(std::unique_ptr<Shape>& shp)
{
  ShapeID id;

  if (!shapeFreeID.empty())
  {
    id = shapeFreeID.back();
    shapeFreeID.pop_back();
  }
  else 
  {
    id = nextShapeFreeID++;
  }

  shapeIDToVec[id] = shapes.size();
  shapes.push_back(std::move(shp));

  return id;
}

BodyID World::createBody(const Body& body)
{
  BodyID id;

  if (!bodyFreeID.empty())
  {
    id = bodyFreeID.back();
    bodyFreeID.pop_back();
  }
  else
  {
    id = nextBodyFreeID++;
  }

  bodyIDToVec[id] = bodies.size();
  bodies.push_back(body);

  return id;
}
