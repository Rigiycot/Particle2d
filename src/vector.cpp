#include "particle2d/vector.hpp"

#include <cmath>

AngleGrad::AngleGrad(float angle)
  : grad(angle)
{}

AngleGrad::AngleGrad(const AngleRad& angle)
  : grad(angle.toGrad().grad)
{}

AngleRad::AngleRad(float angle)
  : rad(angle)
{}

AngleRad::AngleRad(const AngleGrad& angle)
  : rad(angle.toRad().rad)
{}

AngleGrad AngleRad::toGrad()
const {
  return AngleGrad(this->rad * (180.0f / M_PI));
}

AngleRad AngleGrad::toRad()
const {
  return AngleRad(this->grad * (M_PI / 180.0f));
}


Vec2::Vec2(const float x, const float y)
{
  this->x = x;
  this->y = y;
}

float Vec2::length()
const {
  return std::sqrtf(this->lengthSquared());
}

float Vec2::lengthSquared()
const {
  return this->x * this->x + this->y * this->y;
}

Vec2 Vec2::norm()
const {
  float len = this->length();
  if (len < 1e-6f)
    return {0, 0};
  return *this / len;
}

Vec2 Vec2::perpendicular()
const {
  return {-y, x};
}


Vec2 Vec2::operator+(const Vec2& other)
const {
  return {this->x + other.x, this->y + other.y};
}

Vec2 Vec2::operator-(const Vec2& other)
const {
  return {this->x - other.x, this->y - other.y};
}

Vec2 Vec2::operator*(const float scalar)
const {
  return {this->x * scalar, this->y * scalar};
}

Vec2 Vec2::operator/(const float scalar)
const {
  return {this->x / scalar, this->y / scalar};
}

Vec2 Vec2::operator-()
const {
  return {-this->x, -this->y};
}

Vec2& Vec2::operator+=(const Vec2& other)
{
  this->x += other.x;
  this->y += other.y;
  return *this;
}

Vec2& Vec2::operator-=(const Vec2& other)
{
  this->x -= other.x;
  this->y -= other.y;
  return *this;
}

Vec2& Vec2::operator*=(const float scalar)
{
  this->x *= scalar;
  this->y *= scalar;
  return *this;
}

Vec2& Vec2::operator/=(const float scalar)
{
  this->x /= scalar;
  this->y /= scalar;
  return *this;
}

bool Vec2::operator==(const Vec2& other)
const {
  return (
    this->x == other.x &&
    this->y == other.y
  );
}

bool Vec2::operator!=(const Vec2& other)
const {
  return !(*this == other);
}

bool Vec2::equal(const Vec2& other, const float epsilon)
const {
  return (*this - other).lengthSquared() <= epsilon * epsilon;
}

bool Vec2::nequal(const Vec2& other, const float epsilon)
const {
  return (*this - other).lengthSquared() > epsilon * epsilon;
}

Vec2 Vec2::rotate(const AngleRad&  angle)
const {
  return {
    this->x * cosf(angle.rad) - this->y * sinf(angle.rad),
    this->x * sinf(angle.rad) + this->y * cosf(angle.rad)
  };
}

Vec2 Vec2::rotate(const AngleGrad& angle)
const {
  return this->rotate(angle.toRad());
}


float dot(const Vec2& a, const Vec2& b)
{
  return a.x * b.x + a.y * b.y;
}

float cross(const Vec2& a, const Vec2& b)
{
  return a.x * b.y - a.y * b.x;
}
