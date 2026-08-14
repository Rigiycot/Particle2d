#include "particle2d/solver/solver.hpp"
#include "particle2d/joint/joint.hpp"
#include "particle2d/particle/shape.hpp"
#include "particle2d/vector.hpp"
#include "particle2d/world/world.hpp"
#include "particle2d/particle/particle.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <utility> 
#include <cfloat>
#include <algorithm>

void Solver::integrate(World& world, float dt)
{
  for (Particle& p : world.particles)
  {
    Vec2 velocity = p.pos - p.prevPos;

    p.prevPos = p.pos;

    p.pos += velocity;
    p.pos += world.gravity * dt * dt;
  }
}

void Solver::constraints(World& world, float dt, uint16_t iterations)
{
  for (Joint& joint : world.joints)
  {
    Particle& a = world.getParticle(joint.a);
    Particle& b = world.getParticle(joint.b);

    float wA = a.inverseMass;
    float wB = b.inverseMass;

    float sum = wA + wB;

    if (sum == 0.0f)
      continue;

    float k = powf(joint.shiftness, 1.0f / iterations);
    
    Vec2 delta = a.pos - b.pos;
    
    float dist = delta.length();

    if (dist == 0.0f)
      continue;

    Vec2 n = delta / dist;

    float error = dist - joint.length;

    Vec2 correction = n * error;

    a.pos += correction * (wA / sum) * k;
    b.pos -= correction * (wB / sum) * k;
  }

  for (AngleJoint& anglej : world.anglejoints)
  {
    Particle& a = world.getParticle(anglej.a);
    Particle& b = world.getParticle(anglej.b);
    Particle& c = world.getParticle(anglej.c);

    float k = powf(anglej.shiftness, 1.0f / iterations);

    Vec2 ab = a.pos - b.pos;
    Vec2 bc = c.pos - b.pos;

    AngleGrad angle = AngleRad(atan2f(cross(ab, bc), dot(ab, bc)));

    ab = ab.rotate(angle);
    bc = bc.rotate(angle);

    AngleGrad targetAngle;

    if (angle.grad < anglej.minAngle.grad)
      targetAngle = anglej.minAngle;
    else if (angle.grad > anglej.maxAngle.grad)
      targetAngle = anglej.maxAngle;
    else
      continue;

    float delta = targetAngle.grad - angle.grad;

    delta *= k;

    float wA = a.inverseMass;
    float wC = c.inverseMass;

    float sum = wA + wC;

    if (sum == 0.0f)
      continue;

    float correction = delta * k;

    float factorA = wA / sum;
    float factorC = wC / sum;

    Vec2 newAB = ab.rotate(AngleGrad{ correction * factorA});
    Vec2 newBC = bc.rotate(AngleGrad{-correction * factorC});

    a.pos = b.pos + newAB;
    c.pos = b.pos + newBC;
  }
}

void Solver::collisions(World& world, float dt)
{
  for (auto it1 = world.bodies.begin(); it1 != world.bodies.end(); ++it1)
  {
    for (auto it2 = std::next(it1); it2 != world.bodies.end(); ++it2)
    {
      Body& b1 = *it1;
      Body& b2 = *it2;

      if (((b1.collides & b2.category) == 0) || ((b2.collides & b1.category) == 0)) continue;

      Shape& s1 = world.getShape(b1.shape);
      Shape& s2 = world.getShape(b2.shape);
      
      Shape* first = &s1;
      Shape* second = &s2;

      if (first->type > second->type)
        std::swap(first, second);

      switch (first->type)
      {
        case ShapeType::None:
          throw std::runtime_error("Used non-body Shape!");
          break;

        case ShapeType::Rectangle:
          switch (second->type)
          {
            case ShapeType::None:
              {
                throw std::runtime_error("Used non-body Shape!");
                break;
              }

            case ShapeType::Rectangle:
            {
              RectangleShape& rectA = static_cast<RectangleShape&>(*first);
              RectangleShape& rectB = static_cast<RectangleShape&>(*second);
              
              std::array<Vec2, 4> a;
              std::array<Vec2, 4> b;

              for (int i = 0; i < 4; ++i)
              {
                a[i] = world.getParticle(rectA.points[i]).pos;
                b[i] = world.getParticle(rectB.points[i]).pos;
              }

              auto sat = [](const auto& a, const auto& b) -> std::pair<float, Vec2>
              {
                float minOverlap = FLT_MAX;
                Vec2 bestAxis;

                for (const auto& p : {a, b})
                {
                  for (size_t i = 0; i < p.size(); ++i)
                  {
                    Vec2 axis = (p[(i+1) % p.size()] - p[i]).perpendicular().norm();

                    float minA = FLT_MAX; float maxA = -FLT_MAX;
                    float minB = FLT_MAX; float maxB = -FLT_MAX;

                    for (const Vec2& v : a)
                    {
                      float x = dot(v, axis);
                      minA = std::min(minA, x);
                      maxA = std::max(maxA, x);
                    }

                    for (const Vec2& v : b)
                    {
                      float x = dot(v, axis);
                      minB = std::min(minB, x);
                      maxB = std::max(maxB, x);
                    }

                    if (maxA < minB || maxB < minA)
                      return {-1.0f, {}};

                    float overlap = std::min(maxA, maxB) - std::max(minA, minB);

                    if (overlap < minOverlap)
                    {
                      minOverlap = overlap;
                      bestAxis = axis;
                    }
                  }
                }
                return {minOverlap, bestAxis};
              };

              auto [overlap, axis] = sat(a, b);

              if (overlap < 0.0f)
                continue;

              Vec2 centerA = (a[0] + a[1] + a[2] + a[3]) * 0.25f;
              Vec2 centerB = (b[0] + b[1] + b[2] + b[3]) * 0.25f;

              if (dot(centerB - centerA, axis) < 0.0f)
                axis = -axis;

              size_t supportA = 0;
              for (size_t i = 1; i < 4; ++i)
                if (dot(a[i], axis) > dot(a[supportA], axis))
                  supportA = i;

              size_t supportB = 0;
              for (size_t i = 1; i < 4; ++i)
                if (dot(b[i], axis) > dot(b[supportA], axis))
                  supportB = i;


              Particle& pa = world.getParticle(rectA.points[supportA]);
              Particle& pb = world.getParticle(rectB.points[supportB]);

              float wA = pa.inverseMass;
              float wB = pb.inverseMass;
              float sum = wA + wB;

              if (sum == 0.0f)
                continue;

              Vec2 correction = axis * overlap;

              pa.pos -= correction * (wA / sum);
              pb.pos += correction * (wB / sum);
            }

            case ShapeType::Circle:
            {
              RectangleShape& rect = static_cast<RectangleShape&>(*first);
              CircleShape&  circle = static_cast<CircleShape&>(*second);
              Particle& centerP = world.getParticle(circle.center);
              Vec2 center = centerP.pos;
              float radius = circle.radius;

              std::array<Vec2, 4> points;
              for (size_t i = 0; i < 4; ++i)
                points[i] = world.getParticle(rect.points[i]).pos;

              Vec2 newCenter = center;

              Vec2 axis1 = (points[1] - points[0]).perpendicular().norm();
              Vec2 axis2 = (points[3] - points[0]).perpendicular().norm();

              Vec2 bestAxis = axis1;

              float minRect =  FLT_MAX;
              float maxRect = -FLT_MAX;

              float minRect2 =  FLT_MAX;
              float maxRect2 = -FLT_MAX;

              for (const Vec2& point : points)
              {
                float proj = dot(point, axis1);
                float proj2 = dot(point, axis2);

                minRect = std::min(minRect, proj);
                maxRect = std::max(maxRect, proj);

                minRect2 = std::min(minRect2, proj2);
                maxRect2 = std::max(maxRect2, proj2);
              }

              float centerProj =  dot(center, axis1);
              float centerProj2 = dot(center, axis2);
              
              float minCircle = centerProj - radius;
              float maxCircle = centerProj + radius;

              float minCircle2 = centerProj2 - radius;
              float maxCircle2 = centerProj2 + radius;

              if (maxRect < minCircle || minRect > maxCircle || maxRect2 < minCircle2 || minRect2 > maxCircle2)
                break;

              float overlap =  std::min(maxRect,  maxCircle ) - std::max(minRect,  minCircle );
              float overlap2 = std::min(maxRect2, maxCircle2) - std::max(minRect2, minCircle2);

              if (overlap < overlap2)
              {
                overlap  = overlap2;
                bestAxis = axis2;
              }

              float closestDistSq = FLT_MAX;
              Vec2 closestPoint;

              for (size_t i = 0; i < 4; ++i)
              {
                Vec2 a = points[i];
                Vec2 b = points[(i+1) % 4];

                Vec2 edge = b - a;

                float t = dot(center - a, edge) / edge.lengthSquared();

                t = std::clamp(t, 0.0f, 1.0f);

                Vec2 closest = a + edge * t;

                float distSq = (closest - center).lengthSquared();
                float dist   = std::sqrtf(distSq);

                if (distSq < closestDistSq)
                {
                    closestDistSq = distSq;
                    closestPoint = closest;
                }
              }

              Vec2 delta = center - closestPoint;
              float dist = delta.length();

              float closestOverlap = radius - dist;;

              if (dist < radius && dist > 0.0f)
              {
                Vec2 norm = delta.norm();
                float closestOverlap = radius - dist;

                if (closestOverlap < overlap)
                {
                  overlap = closestOverlap;
                  bestAxis = norm;
                }
              }

              Vec2 correction = bestAxis * overlap;

              Vec2 rectCenter;
              for (const Vec2& point : points)
                rectCenter += point;

              rectCenter /= 4.0f;

              if (dot(bestAxis, center - rectCenter) < 0.0f)
                bestAxis = -bestAxis;

              float rectInverseMass = 0.0f;

              for (size_t i = 0; i < 4; ++i)
                rectInverseMass += world.getParticle(rect.points[i]).inverseMass;

              float circleInverseMass = centerP.inverseMass;

              float sum = rectInverseMass + circleInverseMass;

              if (sum == 0.0f)
                break;
              
              Vec2 rectCorrection =   correction * (rectInverseMass   / sum);
              Vec2 circleCorrection = correction * (circleInverseMass / sum);

              for (size_t i = 0; i < 4; ++i)
              {
                Particle& particle = world.getParticle(rect.points[i]);

                if (rectInverseMass > 0.0f)
                  particle.pos -= rectCorrection * (particle.inverseMass / sum);
              }

              newCenter += circleCorrection;

              centerP.pos = newCenter;
            }
          }
      }
    }
  }
}
