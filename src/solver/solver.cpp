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


void p2::Solver::integrate(World& world, float dt)
{
  if (dt <= 0.0f) {
      throw std::runtime_error("Некорректный временной шаг");
  }

    
  for (Particle& p : world.particles)
  {
    if (!std::isfinite(p.pos.x) || !std::isfinite(p.pos.y))
      throw std::runtime_error("NaN before integrate: pos");

    if (!std::isfinite(p.prevPos.x) || !std::isfinite(p.prevPos.y))
      throw std::runtime_error("NaN before integrate: prevPos");

    if (!std::isfinite(p.inverseMass))
      throw std::runtime_error("NaN before integrate: inverseMass");

    if (!std::isfinite(world.gravity.x) ||
      !std::isfinite(world.gravity.y))
      throw std::runtime_error("NaN before integrate: gravity");

    if (!std::isfinite(dt))
      throw std::runtime_error("NaN before integrate: dt");

    if (!p.isActive)
        continue;

    if (p.inverseMass < 0.0f) {
        throw std::runtime_error("Некорректная inverseMass");
    }

    Vec2 prev = p.pos;
    Vec2 acceleration = {0.0f, 0.0f};
    if (p.useGravity)
      acceleration += world.gravity;
      
    p.pos = p.pos * 2.0f - prev + acceleration * dt * dt;
  }
}

void p2::Solver::constraints(World& world, float dt, uint16_t iterations)
{
  for (Joint& joint : world.joints)
  {
    Particle& a = world.getParticle(joint.a);
    Particle& b = world.getParticle(joint.b);

    if (!a.isActive || !b.isActive)
      continue;

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

    if (!std::isfinite(dist))
      throw std::runtime_error("Infinite Joint distance");

    Vec2 n = delta / dist;

    float error = dist - joint.length;

    Vec2 correction = n * error * k;

    a.pos -= correction * (wA / sum);
    b.pos += correction * (wB / sum);
  }

  for (AngleJoint& anglej : world.anglejoints)
  {
    Particle& a = world.getParticle(anglej.a);
    Particle& b = world.getParticle(anglej.b);
    Particle& c = world.getParticle(anglej.c);

    if (!a.isActive || !b.isActive || !c.isActive)
      continue;

    float k = powf(anglej.shiftness, 1.0f / iterations);

    Vec2 ab = a.pos - b.pos;
    Vec2 bc = c.pos - b.pos;

    AngleGrad angle = AngleRad(
      atan2f(cross(ab, bc), dot(ab, bc))
    );

    AngleGrad targetAngle;

    if (angle.grad < anglej.minAngle.grad)
      targetAngle = anglej.minAngle;
    else if (angle.grad > anglej.maxAngle.grad)
      targetAngle = anglej.maxAngle;
    else
      continue;

    float delta = (targetAngle.grad - angle.grad) * k;

    float wA = a.inverseMass;
    float wC = c.inverseMass;

    float sum = wA + wC;

    if (sum == 0.0f)
      continue;

    float factorA = wA / sum;
    float factorC = wC / sum;

    Vec2 newAB = ab.rotate(
      AngleGrad{-delta * factorA}
    );

    Vec2 newBC = bc.rotate(
      AngleGrad{delta * factorC}
    );

    a.pos = b.pos + newAB;
    c.pos = b.pos + newBC;
  }
}

void p2::Solver::collisions(World& world, float dt)
{
  for (auto it1 = world.bodies.begin(); it1 != world.bodies.end(); ++it1)
  {
    for (auto it2 = std::next(it1); it2 != world.bodies.end(); ++it2)
    {
      Body& b1 = *it1;
      Body& b2 = *it2;

      if (((b1.collides & b2.category) == 0) ||
          ((b2.collides & b1.category) == 0))
        continue;

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

        case ShapeType::Rectangle:
        {
          switch (second->type)
          {
            case ShapeType::None:
              throw std::runtime_error("Used non-body Shape!");

            case ShapeType::Rectangle:
            {
              RectangleShape& rectA =
                static_cast<RectangleShape&>(*first);

              RectangleShape& rectB =
                static_cast<RectangleShape&>(*second);

              std::array<Vec2, 4> a;
              std::array<Vec2, 4> b;

              bool anyInactive = false;

              for (size_t i = 0; i < 4; ++i)
              {
                Particle& pA =
                  world.getParticle(rectA.points[i]);

                Particle& pB =
                  world.getParticle(rectB.points[i]);

                a[i] = pA.pos;
                b[i] = pB.pos;

                if (!pA.isActive || !pB.isActive)
                {
                  anyInactive = true;
                  break;
                }
              }

              if (anyInactive)
                break;

              auto sat =
                [](const auto& a, const auto& b)
                -> std::pair<float, Vec2>
              {
                float minOverlap = FLT_MAX;
                Vec2 bestAxis;

                for (const auto& polygon : {a, b})
                {
                  for (size_t i = 0; i < polygon.size(); ++i)
                  {
                    Vec2 edge =
                      polygon[(i + 1) % polygon.size()] -
                      polygon[i];

                    if (edge.lengthSquared() == 0.0f)
                      continue;

                    Vec2 axis = edge.perpendicular().norm();

                    float minA = FLT_MAX;
                    float maxA = -FLT_MAX;

                    float minB = FLT_MAX;
                    float maxB = -FLT_MAX;

                    for (const Vec2& v : a)
                    {
                      float p = dot(v, axis);

                      minA = std::min(minA, p);
                      maxA = std::max(maxA, p);
                    }

                    for (const Vec2& v : b)
                    {
                      float p = dot(v, axis);

                      minB = std::min(minB, p);
                      maxB = std::max(maxB, p);
                    }

                    if (maxA < minB || maxB < minA)
                      return {-1.0f, {}};

                    float overlap =
                      std::min(maxA, maxB) -
                      std::max(minA, minB);

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
                break;

              Vec2 centerA =
                (a[0] + a[1] + a[2] + a[3]) * 0.25f;

              Vec2 centerB =
                (b[0] + b[1] + b[2] + b[3]) * 0.25f;

              if (dot(centerB - centerA, axis) < 0.0f)
                axis = -axis;

              size_t supportA = 0;
              size_t supportB = 0;

              for (size_t i = 1; i < 4; ++i)
              {
                if (dot(a[i], axis) > dot(a[supportA], axis))
                  supportA = i;

                if (dot(b[i], axis) > dot(b[supportB], axis))
                  supportB = i;
              }

              Particle& pa =
                world.getParticle(rectA.points[supportA]);

              Particle& pb =
                world.getParticle(rectB.points[supportB]);

              float wA = pa.inverseMass;
              float wB = pb.inverseMass;

              float sum = wA + wB;

              if (sum == 0.0f)
                break;

              Vec2 correction = axis * overlap;

              pa.pos -= correction * (wA / sum);
              pb.pos += correction * (wB / sum);

              break;
            }

            case ShapeType::Circle:
            {
              RectangleShape& rect =
                static_cast<RectangleShape&>(*first);

              CircleShape& circle =
                static_cast<CircleShape&>(*second);

              Particle& centerP =
                world.getParticle(circle.center);

              if (!centerP.isActive)
                break;

              std::array<Vec2, 4> points;

              bool inactive = false;

              for (size_t i = 0; i < 4; ++i)
              {
                Particle& p =
                  world.getParticle(rect.points[i]);

                if (!p.isActive)
                {
                  inactive = true;
                  break;
                }

                points[i] = p.pos;
              }

              if (inactive)
                break;

              Vec2 center = centerP.pos;
              float radius = circle.radius;

              Vec2 axis1 =
                (points[1] - points[0]).perpendicular().norm();

              Vec2 axis2 =
                (points[3] - points[0]).perpendicular().norm();

              if (axis1.lengthSquared() == 0.0f ||
                  axis2.lengthSquared() == 0.0f)
                break;

              float minRect = FLT_MAX;
              float maxRect = -FLT_MAX;

              float minRect2 = FLT_MAX;
              float maxRect2 = -FLT_MAX;

              for (const Vec2& point : points)
              {
                float p1 = dot(point, axis1);
                float p2 = dot(point, axis2);

                minRect = std::min(minRect, p1);
                maxRect = std::max(maxRect, p1);

                minRect2 = std::min(minRect2, p2);
                maxRect2 = std::max(maxRect2, p2);
              }

              float centerProj = dot(center, axis1);
              float centerProj2 = dot(center, axis2);

              float minCircle = centerProj - radius;
              float maxCircle = centerProj + radius;

              float minCircle2 = centerProj2 - radius;
              float maxCircle2 = centerProj2 + radius;

              if (maxRect < minCircle ||
                  minRect > maxCircle ||
                  maxRect2 < minCircle2 ||
                  minRect2 > maxCircle2)
                break;

              float overlap =
                std::min(maxRect, maxCircle) -
                std::max(minRect, minCircle);

              float overlap2 =
                std::min(maxRect2, maxCircle2) -
                std::max(minRect2, minCircle2);

              Vec2 bestAxis = axis1;

              if (overlap2 < overlap)
              {
                overlap = overlap2;
                bestAxis = axis2;
              }

              float closestDistSq = FLT_MAX;
              Vec2 closestPoint;

              for (size_t i = 0; i < 4; ++i)
              {
                Vec2 a = points[i];
                Vec2 b = points[(i + 1) % 4];

                Vec2 edge = b - a;

                float lengthSq = edge.lengthSquared();

                if (lengthSq == 0.0f)
                  continue;

                float t =
                  dot(center - a, edge) / lengthSq;

                t = std::clamp(t, 0.0f, 1.0f);

                Vec2 closest = a + edge * t;

                float distSq =
                  (closest - center).lengthSquared();

                if (distSq < closestDistSq)
                {
                  closestDistSq = distSq;
                  closestPoint = closest;
                }
              }

              bool inside =
                centerProj >= minRect &&
                centerProj <= maxRect &&
                centerProj2 >= minRect2 &&
                centerProj2 <= maxRect2;

              Vec2 delta = center - closestPoint;
              float dist = delta.length();

              if (inside)
              {
                if (dist > 0.0f)
                {
                  Vec2 normal = -delta / dist;
                  float candidate = radius + dist;

                  if (candidate < overlap)
                  {
                    overlap = candidate;
                    bestAxis = normal;
                  }
                }
              }
              else if (dist < radius && dist > 0.0f)
              {
                Vec2 normal = delta / dist;
                float candidate = radius - dist;

                if (candidate < overlap)
                {
                  overlap = candidate;
                  bestAxis = normal;
                }
              }

              Vec2 rectCenter;

              for (const Vec2& point : points)
                rectCenter += point;

              rectCenter /= 4.0f;

              if (dot(bestAxis, center - rectCenter) < 0.0f)
                bestAxis = -bestAxis;

              float rectInverseMass = 0.0f;

              for (size_t i = 0; i < 4; ++i)
              {
                rectInverseMass +=
                  world.getParticle(rect.points[i]).inverseMass;
              }

              float circleInverseMass =
                centerP.inverseMass;

              float sum =
                rectInverseMass + circleInverseMass;

              if (sum == 0.0f)
                break;

              Vec2 correction = bestAxis * overlap;

              Vec2 rectCorrection =
                correction * (rectInverseMass / sum);

              Vec2 circleCorrection =
                correction * (circleInverseMass / sum);

              if (rectInverseMass > 0.0f)
              {
                for (size_t i = 0; i < 4; ++i)
                {
                  Particle& p =
                    world.getParticle(rect.points[i]);

                  p.pos -=
                    rectCorrection *
                    (p.inverseMass / rectInverseMass);
                }
              }

              if (circleInverseMass > 0.0f)
                centerP.pos += circleCorrection;

              break;
            }

            case ShapeType::Capsule:
            {
              RectangleShape& rect =
                static_cast<RectangleShape&>(*first);

              CapsuleShape& capsule =
                static_cast<CapsuleShape&>(*second);

              Particle& aP =
                world.getParticle(capsule.a);

              Particle& bP =
                world.getParticle(capsule.b);

              if (!aP.isActive || !bP.isActive)
                break;

              float rectInverseMass = 0.0f;
              bool inactive = false;

              std::array<Vec2, 4> points;

              for (size_t i = 0; i < 4; ++i)
              {
                Particle& p =
                  world.getParticle(rect.points[i]);

                if (!p.isActive)
                {
                  inactive = true;
                  break;
                }

                points[i] = p.pos;
                rectInverseMass += p.inverseMass;
              }

              if (inactive)
                break;

              float capsuleInverseMass =
                aP.inverseMass + bP.inverseMass;

              float sum =
                rectInverseMass + capsuleInverseMass;

              if (sum == 0.0f)
                break;

              Vec2 a = aP.pos;
              Vec2 b = bP.pos;

              float radius = capsule.radius;

              Vec2 axis1 =
                (points[1] - points[0]).perpendicular();

              Vec2 axis2 =
                (points[3] - points[0]).perpendicular();

              float overlap = FLT_MAX;
              Vec2 bestAxis;

              auto testAxis =
                [&](Vec2 axis) -> bool
              {
                if (axis.lengthSquared() == 0.0f)
                  return true;

                axis = axis.norm();

                float minRect = FLT_MAX;
                float maxRect = -FLT_MAX;

                for (const Vec2& point : points)
                {
                  float p = dot(point, axis);

                  minRect = std::min(minRect, p);
                  maxRect = std::max(maxRect, p);
                }

                float projA = dot(a, axis);
                float projB = dot(b, axis);

                float minCapsule =
                  std::min(projA, projB) - radius;

                float maxCapsule =
                  std::max(projA, projB) + radius;

                if (maxRect < minCapsule ||
                    minRect > maxCapsule)
                  return false;

                float currentOverlap =
                  std::min(maxRect, maxCapsule) -
                  std::max(minRect, minCapsule);

                if (currentOverlap < overlap)
                {
                  overlap = currentOverlap;
                  bestAxis = axis;
                }

                return true;
              };

              if (!testAxis(axis1) ||
                  !testAxis(axis2))
                break;

              if (!testAxis(b - a))
                break;

              bool collision = true;

              for (const Vec2& point : points)
              {
                if (!testAxis(point - a) ||
                    !testAxis(point - b))
                {
                  collision = false;
                  break;
                }
              }

              if (!collision)
                break;

              Vec2 rectCenter;

              for (const Vec2& point : points)
                rectCenter += point;

              rectCenter /= 4.0f;

              Vec2 capsuleCenter =
                (a + b) * 0.5f;

              if (dot(bestAxis,
                      capsuleCenter - rectCenter) < 0.0f)
                bestAxis = -bestAxis;

              Vec2 correction =
                bestAxis * overlap;

              Vec2 rectCorrection =
                correction *
                (rectInverseMass / sum);

              Vec2 capsuleCorrection =
                correction *
                (capsuleInverseMass / sum);

              if (rectInverseMass > 0.0f)
              {
                for (size_t i = 0; i < 4; ++i)
                {
                  Particle& p =
                    world.getParticle(rect.points[i]);

                  p.pos -=
                    rectCorrection *
                    (p.inverseMass / rectInverseMass);
                }
              }

              if (capsuleInverseMass > 0.0f)
              {
                aP.pos +=
                  capsuleCorrection *
                  (aP.inverseMass / capsuleInverseMass);

                bP.pos +=
                  capsuleCorrection *
                  (bP.inverseMass / capsuleInverseMass);
              }

              break;
            }

            default:
              break;
          }

          break;
        }

        case ShapeType::Circle:
        {
          switch (second->type)
          {
            case ShapeType::Circle:
            {
              CircleShape& circle1 =
                static_cast<CircleShape&>(*first);

              CircleShape& circle2 =
                static_cast<CircleShape&>(*second);

              Particle& c1 =
                world.getParticle(circle1.center);

              Particle& c2 =
                world.getParticle(circle2.center);

              if (!c1.isActive || !c2.isActive)
                break;

              Vec2 delta = c2.pos - c1.pos;

              float distSq =
                delta.lengthSquared();

              float radiusSum =
                circle1.radius + circle2.radius;

              if (distSq >= radiusSum * radiusSum)
                break;

              if (distSq == 0.0f)
                break;

              float dist = std::sqrtf(distSq);

              Vec2 normal =
                delta / dist;

              float overlap =
                radiusSum - dist;

              float sum =
                c1.inverseMass + c2.inverseMass;

              if (sum == 0.0f)
                break;

              Vec2 correction =
                normal * overlap;

              c1.pos -=
                correction *
                (c1.inverseMass / sum);

              c2.pos +=
                correction *
                (c2.inverseMass / sum);

              break;
            }

            case ShapeType::Capsule:
            {
              CircleShape& circle =
                static_cast<CircleShape&>(*first);

              CapsuleShape& capsule =
                static_cast<CapsuleShape&>(*second);

              Particle& c =
                world.getParticle(circle.center);

              Particle& cc1 =
                world.getParticle(capsule.a);

              Particle& cc2 =
                world.getParticle(capsule.b);

              if (!c.isActive ||
                  !cc1.isActive ||
                  !cc2.isActive)
                break;

              Vec2 center = c.pos;
              Vec2 a = cc1.pos;
              Vec2 b = cc2.pos;

              Vec2 edge = b - a;
              float edgeLengthSq =
                edge.lengthSquared();

              Vec2 closest;

              if (edgeLengthSq == 0.0f)
              {
                closest = a;
              }
              else
              {
                float t =
                  dot(center - a, edge) /
                  edgeLengthSq;

                t = std::clamp(t, 0.0f, 1.0f);

                closest =
                  a + edge * t;
              }

              Vec2 delta =
                center - closest;

              float distSq =
                delta.lengthSquared();

              float radiusSum =
                circle.radius + capsule.radius;

              if (distSq >= radiusSum * radiusSum)
                break;

              float dist = std::sqrtf(distSq);

              Vec2 normal;

              if (dist > 0.0f)
              {
                normal =
                  delta / dist;
              }
              else
              {
                Vec2 capsuleCenter =
                  (a + b) * 0.5f;

                Vec2 fallback =
                  center - capsuleCenter;

                if (fallback.lengthSquared() > 0.0f)
                  normal = fallback.norm();
                else if (edgeLengthSq > 0.0f)
                  normal = edge.perpendicular().norm();
                else
                  normal = Vec2(1.0f, 0.0f);
              }

              float overlap =
                radiusSum - dist;

              float circleInverseMass =
                c.inverseMass;

              float capsuleInverseMass =
                cc1.inverseMass +
                cc2.inverseMass;

              float sum =
                circleInverseMass +
                capsuleInverseMass;

              if (sum == 0.0f)
                break;

              Vec2 correction =
                normal * overlap;

              Vec2 circleCorrection =
                correction *
                (circleInverseMass / sum);

              Vec2 capsuleCorrection =
                correction *
                (capsuleInverseMass / sum);

              if (circleInverseMass > 0.0f)
                c.pos += circleCorrection;

              if (capsuleInverseMass > 0.0f)
              {
                cc1.pos -=
                  capsuleCorrection *
                  (cc1.inverseMass /
                   capsuleInverseMass);

                cc2.pos -=
                  capsuleCorrection *
                  (cc2.inverseMass /
                   capsuleInverseMass);
              }

              break;
            }

            default:
              break;
          }

          break;
        }

        case ShapeType::Capsule:
        {
          CapsuleShape& capsule1 =
            static_cast<CapsuleShape&>(*first);

          CapsuleShape& capsule2 =
            static_cast<CapsuleShape&>(*second);

          Particle& a1P =
            world.getParticle(capsule1.a);

          Particle& a2P =
            world.getParticle(capsule1.b);

          Particle& b1P =
            world.getParticle(capsule2.a);

          Particle& b2P =
            world.getParticle(capsule2.b);

          if (!a1P.isActive ||
              !a2P.isActive ||
              !b1P.isActive ||
              !b2P.isActive)
            break;

          float inverseMass1 =
            a1P.inverseMass +
            a2P.inverseMass;

          float inverseMass2 =
            b1P.inverseMass +
            b2P.inverseMass;

          float sum =
            inverseMass1 + inverseMass2;

          if (sum == 0.0f)
            break;

          Vec2 a = a1P.pos;
          Vec2 b = a2P.pos;

          Vec2 c = b1P.pos;
          Vec2 d = b2P.pos;

          Vec2 ab = b - a;
          Vec2 cd = d - c;
          Vec2 ac = a - c;

          float A = dot(ab, ab);
          float E = dot(cd, cd);

          float s = 0.0f;
          float t = 0.0f;

          if (A == 0.0f && E == 0.0f)
          {
            s = 0.0f;
            t = 0.0f;
          }
          else if (A == 0.0f)
          {
            t = std::clamp(
              dot(a - c, cd) / E,
              0.0f,
              1.0f
            );
          }
          else if (E == 0.0f)
          {
            s = std::clamp(
              dot(c - a, ab) / A,
              0.0f,
              1.0f
            );
          }
          else
          {
            float B = dot(ab, cd);
            float C = dot(ab, ac);
            float F = dot(cd, ac);

            float denom =
              A * E - B * B;

            if (denom != 0.0f)
            {
              s = std::clamp(
                (B * F - C * E) / denom,
                0.0f,
                1.0f
              );
            }

            t =
              (B * s + F) / E;

            if (t < 0.0f)
            {
              t = 0.0f;

              s = std::clamp(
                -C / A,
                0.0f,
                1.0f
              );
            }
            else if (t > 1.0f)
            {
              t = 1.0f;

              s = std::clamp(
                (B - C) / A,
                0.0f,
                1.0f
              );
            }
          }

          Vec2 closest1 =
            a + ab * s;

          Vec2 closest2 =
            c + cd * t;

          Vec2 delta =
            closest2 - closest1;

          float distSq =
            delta.lengthSquared();

          float radiusSum =
            capsule1.radius +
            capsule2.radius;

          if (distSq >= radiusSum * radiusSum)
            break;

          float dist =
            std::sqrtf(distSq);

          Vec2 normal;

          if (dist > 0.0f)
          {
            normal =
              delta / dist;
          }
          else
          {
            Vec2 center1 =
              (a + b) * 0.5f;

            Vec2 center2 =
              (c + d) * 0.5f;

            Vec2 centerDelta =
              center2 - center1;

            if (centerDelta.lengthSquared() > 0.0f)
            {
              normal =
                centerDelta.norm();
            }
            else if (ab.lengthSquared() > 0.0f)
            {
              normal =
                ab.perpendicular().norm();
            }
            else if (cd.lengthSquared() > 0.0f)
            {
              normal =
                cd.perpendicular().norm();
            }
            else
            {
              normal =
                Vec2(1.0f, 0.0f);
            }
          }

          float overlap =
            radiusSum - dist;

          Vec2 correction =
            normal * overlap;

          Vec2 correction1 =
            correction *
            (inverseMass1 / sum);

          Vec2 correction2 =
            correction *
            (inverseMass2 / sum);

          if (inverseMass1 > 0.0f)
          {
            a1P.pos -=
              correction1 *
              (a1P.inverseMass /
               inverseMass1);

            a2P.pos -=
              correction1 *
              (a2P.inverseMass /
               inverseMass1);
          }

          if (inverseMass2 > 0.0f)
          {
            b1P.pos +=
              correction2 *
              (b1P.inverseMass /
               inverseMass2);

            b2P.pos +=
              correction2 *
              (b2P.inverseMass /
               inverseMass2);
          }

          break;
        }
      }
    }
  }
}
