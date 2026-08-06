#pragma once

#include "particle2d/particle/particle.hpp"
#include "particle2d/joint/joint.hpp"
#include "particle2d/particle/shape.hpp"
#include "particle2d/types.hpp"

#include "particle2d/solver/solver.hpp"

#include <unordered_map>
#include <vector>
#include <cstdint>
#include <memory>

class World 
{
  public:
    void step(float dt = 1.0f / 60.0f, uint8_t iterations = 3);

    ParticleID createParticle(const Particle& prt);
    JointID    createJoint(const Joint& jnt);
    ShapeID    createShape(std::unique_ptr<Shape>& shp);

    void addVelocity(ParticleID id, const Vec2& velocity, float dt);

    Particle&   getParticle(ParticleID id);
    Joint&      getJoint(JointID id);
    AngleJoint& getAngleJoint(AngleJointID id);
    Shape&      getShape(ShapeID id);

    World();

    Vec2 gravity;
  private:
    friend class Solver;
    Solver solver;

    std::vector<Particle>                    particles;
    std::unordered_map<ParticleID, uint32_t> particleIdxToVec;
    std::vector<ParticleID>                  particleFreeID;
    ParticleID                               nextParticleFreeID = 0;
    
    std::vector<Joint>                    joints;
    std::unordered_map<JointID, uint32_t> jointIdxToVec;
    std::vector<JointID>                  jointFreeID;
    JointID                               nextJointFreeID = 0;

    std::vector<AngleJoint>               anglejoints;
    std::unordered_map<JointID, uint32_t> anglejointIdxToVec;
    std::vector<JointID>                  anglejointFreeID;
    JointID                               nextAngleJointFreeID = 0;
 

    std::vector<std::unique_ptr<Shape>>   shapes;
    std::unordered_map<ShapeID, uint32_t> shapeIdxToVec;
    std::vector<ShapeID>                  shapeFreeID;
    ShapeID                               nextShapeFreeID = 0;
};
