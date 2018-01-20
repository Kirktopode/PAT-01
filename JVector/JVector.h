#ifndef JVector_h_
#define JVector_h_

#if defined (ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
  #include <pins_arduino.h>
#endif

#if defined (__AVR__)
	#include <avr/io.h>
	#include <avr/interrupt.h>
#endif

class JVector {
  private:
    float x;
    float y;
    float z;
  public:
    JVector(float x, float y, float z);
    JVector(float x, float y);
    JVector();
    JVector operator=(JVector other);
    JVector operator+=(JVector other);
    JVector operator-=(JVector other);
    JVector operator+(JVector other);
    JVector operator-(JVector other);
    JVector operator*(float m);
    JVector operator/(float d);
    JVector cross(JVector other);
    float dot(JVector other);
    float norm();
    float getX();
    float getY();
    float getZ();
    void setX(float x);
    void setY(float x);
    void setZ(float x);
    void print();
};

#endif
