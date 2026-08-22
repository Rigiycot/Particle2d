#pragma once

#include "particle2d/particle/particle.hpp"
#include "particle2d/joint/joint.hpp"
#include "particle2d/particle/shape.hpp"
#include "particle2d/types.hpp"

#include "particle2d/solver/solver.hpp"
#include "particle2d/vector.hpp"
#include "particle2d/world/intersects.hpp"

#include <sys/types.h>
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <memory>

namespace p2
{
  class World 
  {
    public:
      void step(float dt = 1.0f / 60.0f, uint8_t iterations = 3);

      ParticleID   createParticle(const Particle& prt);
      JointID      createJoint(const Joint& jnt);
      AngleJointID createAngleJoint(const AngleJoint& ajnt);
      ShapeID      createShape(std::unique_ptr<Shape> shp);
      BodyID       createBody(const Body& body);

      void addVelocity(ParticleID id, const Vec2& velocity, float dt);
      void dampVelocity(ParticleID id, float amount);

      Vec2 rotate(ParticleID p, float angle);

      Particle&   getParticle(ParticleID id);
      Joint&      getJoint(JointID id);
      AngleJoint& getAngleJoint(AngleJointID id);
      Shape&      getShape(ShapeID id);
      Body&       getBody(BodyID id);

      p2::Collision collide(ShapeID a, ShapeID b);
      bool intersects(ShapeID a, ShapeID b);

      ShapeID createRectangle(
        Vec2 center,
        Vec2 halfSize,
        AngleRad rotation,
        float inverseMass,
        uint64_t category,
        uint64_t collides,
        float friction = 0.8
      );

      ShapeID createRectangle(
        Vec2 center,
        Vec2 halfSize,
        AngleGrad rotation,
        float inverseMass,
        uint64_t category,
        uint64_t collides,
        float friction = 0.8
      );

      ShapeID createRectangle(
        ParticleID p0,
        ParticleID p1,
        ParticleID p2,
        ParticleID p3,
        uint64_t category,
        uint64_t collides,
        float friction = 0.8
      );

      ShapeID createCircle(
        Vec2 center,
        float radius,
        float inverseMass,
        uint64_t category,
        uint64_t collides,
        float friction = 0.8
      );

      ShapeID createCircle(
        ParticleID center,
        float radius,
        uint64_t category,
        uint64_t collides,
        float friction = 0.8
      );

      ShapeID createCapsule(
        Vec2 point1,
        Vec2 point2,
        float radius,
        float inverseMass,
        uint64_t category,
        uint64_t collides,
        float friction = 0.8
      );

      ShapeID createCapsule(
        ParticleID point1,
        ParticleID point2,
        float radius,
        uint64_t category,
        uint64_t collides,
        float friction = 0.8
      );

      ShapeID createCapsule(
        Vec2 center,
        float length,
        AngleRad rotation,
        float radius,
        float inverseMass,
        uint64_t category,
        uint64_t collides,
        float friction = 0.8
      );

      ShapeID createCapsule(
        Vec2 center,
        float length,
        AngleGrad rotation,
        float radius,
        float inverseMass,
        uint64_t category,
        uint64_t collides,
        float friction = 0.8
      );

      World() = default;

      Vec2 gravity;
      float airFriction = 0.99;
    private:
      friend class Solver;
      Solver solver;

      void dampVelocity(Particle& prt, float amount);


      std::vector<Particle>                    particles;
      std::unordered_map<ParticleID, uint32_t> particleIDToVec;
      std::vector<ParticleID>                  particleFreeID;
      ParticleID                               nextParticleFreeID = 0;
      
      std::vector<Joint>                    joints;
      std::unordered_map<JointID, uint32_t> jointIDToVec;
      std::vector<JointID>                  jointFreeID;
      JointID                               nextJointFreeID = 0;

      std::vector<AngleJoint>               anglejoints;
      std::unordered_map<JointID, uint32_t> anglejointIDToVec;
      std::vector<JointID>                  anglejointFreeID;
      JointID                               nextAngleJointFreeID = 0;
  

      std::vector<std::unique_ptr<Shape>>   shapes;
      std::unordered_map<ShapeID, uint32_t> shapeIDToVec;
      std::vector<ShapeID>                  shapeFreeID;
      ShapeID                               nextShapeFreeID = 0;

      std::vector<Body> bodies;
      std::unordered_map<BodyID, uint32_t> bodyIDToVec;
      std::vector<BodyID> bodyFreeID;
      BodyID nextBodyFreeID = 0;
  };
}
