#include <math.h>
#include <stdio.h>

#include "JVector.h"

/*
*
* Jameson Richard's vector library, exclusively for 2- and 3-Dimensional vector math. Ain't no copyrights here.
*
*
*/

JVector::JVector(float x, float y, float z):
 x(x), y(y), z(z){
}
JVector::JVector(float x, float y):
 x(x), y(y), z(0){
}
JVector::JVector():
 x(0), y(0), z(0){
}
JVector JVector::operator+(JVector other){
  JVector result(x + other.x, y + other.y, z + other.z);
  return result;
}
JVector JVector::operator=(JVector other){
  x = other.x;
  y = other.y;
  z = other.z;
}
JVector JVector::operator+=(JVector other){
  x += other.x;
  y += other.y;
  z += other.z;
}
JVector JVector::operator-=(JVector other){
  x -= other.x;
  y -= other.y;
  z -= other.z;
}
JVector JVector::operator-(JVector other){
  JVector result(x - other.x, y - other.y, z - other.z);
  return result;
}
JVector JVector::cross(JVector other){
  JVector result(y * other.z - z * other.y, z * other.x - x * other.z, x * other.y - y * other.x);
  return result;
}
float JVector::dot(JVector other){
  return x * other.x + y * other.y + z * other.z;
}
JVector JVector::operator*(float m){
  JVector result(x * m, y * m, z * m);
  return result;
}
JVector JVector::operator/(float d){
  if(d == 0) return *this;
  JVector result(x / d, y / d, z / d);
  return result;
}
float JVector::norm(){
  return sqrt(x * x + y * y + z * z);
}
void JVector::print(){
  printf("(%.2f, %.2f, %.2f)\n", x, y, z);
}

float JVector::getX(){
  return x;
}
float JVector::getY(){
  return y;
}
float JVector::getZ(){
  return z;
}
void JVector::setX(float Lx){
  x = Lx;
}
void JVector::setY(float Ly){
  y = Ly;
}
void JVector::setZ(float Lz){
  z = Lz;
}
