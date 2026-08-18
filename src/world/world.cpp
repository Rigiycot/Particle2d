#include "particle2d/world/world.hpp"
#include "particle2d/joint/joint.hpp"
#include "particle2d/particle/particle.hpp"
#include "particle2d/particle/shape.hpp"
#include "particle2d/types.hpp"
#include "particle2d/vector.hpp"

#include <memory>
#include <stdexcept>
#include <vector>

#include <cmath>

void p2::World::step(float dt, uint8_t iterations)
{
  solver.integrate(*this, dt);

  for (p2::Particle& p : particles)
  {
    if (!std::isfinite(p.pos.x) || !std::isfinite(p.pos.y))
      throw std::runtime_error("NaN after integrate");
  }

  for (uint8_t i = 0; i < iterations; ++i)
  {
    solver.constraints(*this, dt, iterations);
    for (p2::Particle& p : particles)
    {
      if (!std::isfinite(p.pos.x) || !std::isfinite(p.pos.y))
        throw std::runtime_error("NaN after constraints");
    }
    solver.collisions(*this, dt);
    for (p2::Particle& p : particles)
    {
      if (!std::isfinite(p.pos.x) || !std::isfinite(p.pos.y))
        throw std::runtime_error("NaN after collisions");
    }
  }
}

void p2::World::addVelocity(p2::ParticleID id, const p2::Vec2& velocity, float dt)
{
  p2::Particle& prt = getParticle(id);

  prt.prevPos -= velocity * dt;
}

void p2::World::dampVelocity(p2::ParticleID id, float amount)
{
  p2::Particle& prt = getParticle(id);

  p2::Vec2 velocity = prt.pos - prt.prevPos;
  velocity *= amount;

  prt.prevPos = prt.pos - velocity;
}

p2::Vec2 p2::World::rotate(p2::ParticleID id, float angle)
{
  p2::Particle& particle = getParticle(id);

  p2::Vec2 velocity = particle.pos - particle.prevPos;

  velocity = velocity.rotate(AngleGrad{angle});

  particle.prevPos = particle.pos - velocity;

  return velocity;
}

p2::Particle& p2::World::getParticle(p2::ParticleID id)
{
  return particles.at(particleIDToVec.at(id));
}

p2::Joint& p2::World::getJoint(p2::JointID id)
{
  return joints.at(jointIDToVec.at(id));
}

p2::AngleJoint& p2::World::getAngleJoint(p2::AngleJointID id)
{
  return anglejoints.at(anglejointIDToVec.at(id));
}

p2::Shape& p2::World::getShape(p2::ShapeID id)
{
  return *shapes.at(shapeIDToVec.at(id));
}

p2::Body& p2::World::getBody(p2::BodyID id)
{
  return bodies.at(bodyIDToVec.at(id));
}

p2::ParticleID p2::World::createParticle(const p2::Particle& prt)
{
  p2::ParticleID id;

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

  particles.back().prevPos = particles.back().pos;

  return id;
}

p2::JointID p2::World::createJoint(const p2::Joint& jnt)
{
  p2::JointID id;

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

p2::AngleJointID p2::World::createAngleJoint(const p2::AngleJoint& ajnt)
{
  p2::AngleJointID id;

  if (!anglejointFreeID.empty())
  {
    id = anglejointFreeID.back();
    anglejointFreeID.pop_back();
  }
  else
  {
    id = nextAngleJointFreeID++;
  }

  anglejointIDToVec[id] = anglejoints.size();
  anglejoints.push_back(ajnt);

  return id;
}

p2::ShapeID p2::World::createShape(std::unique_ptr<p2::Shape> shp)
{
  if (!shp)
    throw std::runtime_error("Cannot create null p2::Shape!");

  p2::ShapeID id;

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

p2::BodyID p2::World::createBody(const p2::Body& body)
{
  p2::BodyID id;

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

p2::ShapeID p2::World::createRectangle(
  p2::Vec2 center,
  p2::Vec2 halfSize,
  AngleRad rotation,
  float inverseMass,
  uint64_t category,
  uint64_t collides)
{
  p2::Vec2 p0{-halfSize.x, -halfSize.y};
  p2::Vec2 p1{ halfSize.x, -halfSize.y};
  p2::Vec2 p2{ halfSize.x,  halfSize.y};
  p2::Vec2 p3{-halfSize.x,  halfSize.y};

  p0 = p0.rotate(rotation) + center;
  p1 = p1.rotate(rotation) + center;
  p2 = p2.rotate(rotation) + center;
  p3 = p3.rotate(rotation) + center;

  float particleInverseMass = inverseMass * 0.25f;

  p2::ParticleID id0 = createParticle({p0, particleInverseMass});
  p2::ParticleID id1 = createParticle({p1, particleInverseMass});
  p2::ParticleID id2 = createParticle({p2, particleInverseMass});
  p2::ParticleID id3 = createParticle({p3, particleInverseMass});

  createJoint({
    id0,
    id1,
    (p1 - p0).length(),
    1.0f
  });

  createJoint({
    id1,
    id2,
    (p2 - p1).length(),
    1.0f
  });

  createJoint({
    id2,
    id3,
    (p3 - p2).length(),
    1.0f
  });

  createJoint({
    id3,
    id0,
    (p0 - p3).length(),
    1.0f
  });

  createAngleJoint({
    id0, id1, id2,
    AngleGrad{90.0f},
    AngleGrad{90.0f},
    1.0f
  });

  createAngleJoint({
    id1, id2, id3,
    AngleGrad{90.0f},
    AngleGrad{90.0f},
    1.0f
  });

  createAngleJoint({
    id2, id3, id0,
    AngleGrad{90.0f},
    AngleGrad{90.0f},
    1.0f
  });

  createAngleJoint({
    id3, id0, id1,
    AngleGrad{90.0f},
    AngleGrad{90.0f},
    1.0f
  });

  auto shape = std::make_unique<RectangleShape>();

  shape->points = {
    id0,
    id1,
    id2,
    id3
  };

  p2::ShapeID shapeID = createShape(std::move(shape));

  createBody({
    shapeID,
    category,
    collides
  });

  return shapeID;
};

p2::ShapeID p2::World::createRectangle(
  p2::Vec2 center,
  p2::Vec2 halfSize,
  AngleGrad rotation,
  float inverseMass,
  uint64_t category,
  uint64_t collides)
{
  return this->createRectangle(center, halfSize, rotation.toRad(), inverseMass, category, collides);
}

p2::ShapeID p2::World::createRectangle(
  p2::ParticleID p0,
  p2::ParticleID p1,
  p2::ParticleID p2,
  p2::ParticleID p3,
  uint64_t category,
  uint64_t collides)
{
  getParticle(p0);
  getParticle(p1);
  getParticle(p2);
  getParticle(p3);

  auto shape = std::make_unique<RectangleShape>();

  shape->points = {
    p0,
    p1,
    p2,
    p3
  };

  p2::ShapeID shapeID = createShape(std::move(shape));

  createBody({
    shapeID,
    category,
    collides
  });

  return shapeID;
}

p2::ShapeID p2::World::createCircle(
  p2::Vec2 center,
  float radius,
  float inverseMass,
  uint64_t category,
  uint64_t collides)
{
  p2::ParticleID centerID = createParticle({
    center,
    inverseMass
  });

  return createCircle(
    centerID,
    radius,
    category,
    collides
  );
}

p2::ShapeID p2::World::createCircle(
  p2::ParticleID center,
  float radius,
  uint64_t category,
  uint64_t collides)
{
  getParticle(center);

  auto shape = std::make_unique<CircleShape>();

  shape->center = center;
  shape->radius = radius;

  p2::ShapeID shapeID = createShape(std::move(shape));

  createBody({
    shapeID,
    category,
    collides
  });

  return shapeID;
}

p2::ShapeID p2::World::createCapsule(
  p2::Vec2 point1,
  p2::Vec2 point2,
  float radius,
  float inverseMass,
  uint64_t category,
  uint64_t collides)
{
  p2::ParticleID id1 = createParticle({
    point1,
    inverseMass * 0.5f
  });

  p2::ParticleID id2 = createParticle({
    point2,
    inverseMass * 0.5f
  });

  createJoint({
    id1,
    id2,
    (point2 - point1).length(),
    1.0f
  });

  auto shape = std::make_unique<CapsuleShape>();

  shape->a = id1;
  shape->b = id2;
  shape->radius = radius;

  p2::ShapeID shapeID = createShape(std::move(shape));

  createBody({
    shapeID,
    category,
    collides
  });

  return shapeID;
}

p2::ShapeID p2::World::createCapsule(
  p2::ParticleID point1,
  p2::ParticleID point2,
  float radius,
  uint64_t category,
  uint64_t collides)
{
  getParticle(point1);
  getParticle(point2);

  auto shape = std::make_unique<CapsuleShape>();

  shape->a = point1;
  shape->b = point2;
  shape->radius = radius;

  p2::ShapeID shapeID = createShape(std::move(shape));

  createBody({
    shapeID,
    category,
    collides
  });

  return shapeID;
}

p2::ShapeID p2::World::createCapsule(
  p2::Vec2 center,
  float length,
  AngleRad rotation,
  float radius,
  float inverseMass,
  uint64_t category,
  uint64_t collides)
{
  p2::Vec2 direction = Vec2{1.0f, 0.0f}.rotate(rotation);

  p2::Vec2 offset = direction * (length * 0.5f);

  p2::Vec2 point1 = center - offset;
  p2::Vec2 point2 = center + offset;

  return createCapsule(
    point1,
    point2,
    radius,
    inverseMass,
    category,
    collides
  );
}

p2::ShapeID p2::World::createCapsule(
  p2::Vec2 center,
  float length,
  AngleGrad rotation,
  float radius,
  float inverseMass,
  uint64_t category,
  uint64_t collides)
{
  return createCapsule(
    center,
    length,
    AngleRad{
      rotation.toRad().rad
    },
    radius,
    inverseMass,
    category,
    collides
  );
}
