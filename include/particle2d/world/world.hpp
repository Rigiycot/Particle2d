#pragma once

#include "particle2d/particle/particle.hpp"
#include "particle2d/joint/joint.hpp"
#include "particle2d/particle/shape.hpp"
#include "particle2d/types.hpp"

#include "particle2d/solver/solver.hpp"
#include "particle2d/vector.hpp"

#include <sys/types.h>
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <memory>

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

    /**
    * Creates a rectangle from an axis-aligned bounding box and rotation.
    *
    * @param min Minimum corner of the rectangle before rotation.
    * @param max Maximum corner of the rectangle before rotation.
    * @param rotation Rectangle rotation.
    * @param inverseMass Inverse mass distributed between rectangle particles.
    * @param category Collision category.
    * @param collides Collision mask.
    * @return Automatically assigned ShapeID.
    */
    ShapeID createRectangle(
      Vec2 min,
      Vec2 max,
      AngleRad rotation,
      float inverseMass,
      uint64_t category,
      uint64_t collides
    );

    /**
    * Creates a rectangle from an axis-aligned bounding box and rotation.
    *
    * @param min Minimum corner of the rectangle before rotation.
    * @param max Maximum corner of the rectangle before rotation.
    * @param rotation Rectangle rotation.
    * @param inverseMass Inverse mass distributed between rectangle particles.
    * @param category Collision category.
    * @param collides Collision mask.
    * @return Automatically assigned ShapeID.
    */
    ShapeID createRectangle(
      Vec2 min,
      Vec2 max,
      AngleGrad rotation,
      float inverseMass,
      uint64_t category,
      uint64_t collides
    );

    /**
    * Creates a rectangle using four existing particles.
    *
    * @param p0 First rectangle corner.
    * @param p1 Second rectangle corner.
    * @param p2 Third rectangle corner.
    * @param p3 Fourth rectangle corner.
    * @param category Collision category.
    * @param collides Collision mask.
    * @return Automatically assigned ShapeID.
    */
    ShapeID createRectangle(
      ParticleID p0,
      ParticleID p1,
      ParticleID p2,
      ParticleID p3,
      uint64_t category,
      uint64_t collides
    );

    /**
    * Creates a circle with a newly created center particle.
    *
    * @param center Circle center.
    * @param radius Circle radius.
    * @param inverseMass Inverse mass of the center particle.
    * @param category Collision category.
    * @param collides Collision mask.
    * @return Automatically assigned ShapeID.
    */
    ShapeID createCircle(
      Vec2 center,
      float radius,
      float inverseMass,
      uint64_t category,
      uint64_t collides
    );

    /**
    * Creates a circle using an existing particle as its center.
    *
    * @param center Existing center particle.
    * @param radius Circle radius.
    * @param category Collision category.
    * @param collides Collision mask.
    * @return Automatically assigned ShapeID.
    */
    ShapeID createCircle(
      ParticleID center,
      float radius,
      uint64_t category,
      uint64_t collides
    );

    /**
    * Creates a capsule from two newly created endpoint particles.
    *
    * @param point1 First capsule endpoint.
    * @param point2 Second capsule endpoint.
    * @param radius Capsule radius.
    * @param inverseMass Inverse mass distributed between endpoint particles.
    * @param category Collision category.
    * @param collides Collision mask.
    * @return Automatically assigned ShapeID.
    */
    ShapeID createCapsule(
      Vec2 point1,
      Vec2 point2,
      float radius,
      float inverseMass,
      uint64_t category,
      uint64_t collides
    );

    /**
    * Creates a capsule using two existing particles as endpoints.
    *
    * @param point1 First capsule endpoint.
    * @param point2 Second capsule endpoint.
    * @param radius Capsule radius.
    * @param category Collision category.
    * @param collides Collision mask.
    * @return Automatically assigned ShapeID.
    */
    ShapeID createCapsule(
      ParticleID point1,
      ParticleID point2,
      float radius,
      uint64_t category,
      uint64_t collides
    );

    /**
    * Creates a capsule from its center, length and rotation.
    *
    * @param center Capsule center.
    * @param length Distance between capsule endpoints.
    * @param rotation Capsule rotation.
    * @param radius Capsule radius.
    * @param inverseMass Inverse mass distributed between endpoint particles.
    * @param category Collision category.
    * @param collides Collision mask.
    * @return Automatically assigned ShapeID.
    */
    ShapeID createCapsule(
      Vec2 center,
      float length,
      AngleRad rotation,
      float radius,
      float inverseMass,
      uint64_t category,
      uint64_t collides
    );

    /**
    * Creates a capsule from its center, length and rotation.
    *
    * @param center Capsule center.
    * @param length Distance between capsule endpoints.
    * @param rotation Capsule rotation.
    * @param radius Capsule radius.
    * @param inverseMass Inverse mass distributed between endpoint particles.
    * @param category Collision category.
    * @param collides Collision mask.
    * @return Automatically assigned ShapeID.
    */
    ShapeID createCapsule(
      Vec2 center,
      float length,
      AngleGrad rotation,
      float radius,
      float inverseMass,
      uint64_t category,
      uint64_t collides
    );

    World() = default;

    Vec2 gravity;
  private:
    friend class Solver;
    Solver solver;

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
