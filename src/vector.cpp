#include "particle2d/vector.hpp"

#include <cmath>

p2::AngleGrad::AngleGrad(float angle)
  : grad(angle)
{}

p2::AngleGrad::AngleGrad(const AngleRad& angle)
  : grad(angle.toGrad().grad)
{}

p2::AngleRad::AngleRad(float angle)
  : rad(angle)
{}

p2::AngleRad::AngleRad(const p2::AngleGrad& angle)
  : rad(angle.toRad().rad)
{}

p2::AngleGrad p2::AngleRad::toGrad()
const {
  return p2::AngleGrad(this->rad * (180.0f / M_PI));
}

p2::AngleRad p2::AngleGrad::toRad()
const {
  return AngleRad(this->grad * (M_PI / 180.0f));
}


p2::Vec2::Vec2(const float x, const float y)
{
  this->x = x;
  this->y = y;
}

float p2::Vec2::length()
const {
  return std::sqrtf(this->lengthSquared());
}

float p2::Vec2::lengthSquared()
const {
  return this->x * this->x + this->y * this->y;
}

p2::Vec2 p2::Vec2::norm()
const {
  float len = this->length();
  if (len < 1e-6f)
    return {0, 0};
  return *this / len;
}

p2::Vec2 p2::Vec2::perpendicular()
const {
  return {-y, x};
}


p2::Vec2 p2::Vec2::operator+(const Vec2& other)
const {
  return {this->x + other.x, this->y + other.y};
}

p2::Vec2 p2::Vec2::operator-(const Vec2& other)
const {
  return {this->x - other.x, this->y - other.y};
}

p2::Vec2 p2::Vec2::operator*(const float scalar)
const {
  return {this->x * scalar, this->y * scalar};
}

p2::Vec2 p2::Vec2::operator/(const float scalar)
const {
  return {this->x / scalar, this->y / scalar};
}

p2::Vec2 p2::Vec2::operator-()
const {
  return {-this->x, -this->y};
}

p2::Vec2& p2::Vec2::operator+=(const Vec2& other)
{
  this->x += other.x;
  this->y += other.y;
  return *this;
}

p2::Vec2& p2::Vec2::operator-=(const Vec2& other)
{
  this->x -= other.x;
  this->y -= other.y;
  return *this;
}

p2::Vec2& p2::Vec2::operator*=(const float scalar)
{
  this->x *= scalar;
  this->y *= scalar;
  return *this;
}

p2::Vec2& p2::Vec2::operator/=(const float scalar)
{
  this->x /= scalar;
  this->y /= scalar;
  return *this;
}

bool p2::Vec2::operator==(const Vec2& other)
const {
  return (
    this->x == other.x &&
    this->y == other.y
  );
}

bool p2::Vec2::operator!=(const Vec2& other)
const {
  return !(*this == other);
}

bool p2::Vec2::equal(const Vec2& other, const float epsilon)
const {
  return (*this - other).lengthSquared() <= epsilon * epsilon;
}

bool p2::Vec2::nequal(const Vec2& other, const float epsilon)
const {
  return (*this - other).lengthSquared() > epsilon * epsilon;
}

p2::Vec2 p2::Vec2::rotate(const AngleRad&  angle)
const {
  return {
    this->x * cosf(angle.rad) - this->y * sinf(angle.rad),
    this->x * sinf(angle.rad) + this->y * cosf(angle.rad)
  };
}

p2::Vec2 p2::Vec2::rotate(const p2::AngleGrad& angle)
const {
  return this->rotate(angle.toRad());
}


float p2::dot(const p2::Vec2& a, const p2::Vec2& b)
{
  return a.x * b.x + a.y * b.y;
}

float p2::cross(const p2::Vec2& a, const p2::Vec2& b)
{
  return a.x * b.y - a.y * b.x;
}
