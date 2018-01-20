#include "JVector.h"
#include <stdio.h>
#include <cmath>

const double PI = 3.14159265;

int main(){
  
  JVector pos(0,0);
  JVector waypoints[9] = {
    JVector(0,10), 
    JVector(5,5), 
    JVector(10,0), 
    JVector(5,-5), 
    JVector(0,-10),
    JVector(-5,-5),
    JVector(-10,0),
    JVector(-5,5),
    JVector(0,0)
  };
  
  
  for(int wpIndex = 0; wpIndex < 9; wpIndex++){
    float desiredHeading = atan2( waypoints[wpIndex].getX() - pos.getX() , waypoints[wpIndex].getY() - pos.getY()) * 180 / PI;
    while(desiredHeading < 0) desiredHeading += 360;
    printf("Waypoint: (%.2f, %.2f) Desired Heading: %.2f\n",waypoints[wpIndex].getX(),waypoints[wpIndex].getY(),desiredHeading);
    
    
    for(float heading = 0; heading < 360; heading += 1){
      float headingChange = desiredHeading - heading;
      
      if(headingChange > 180) headingChange -= 360;
      if(headingChange < -180) headingChange += 360;
      printf("Desired Heading: %.2f Heading: %.2f Heading Change: %.2f\n", desiredHeading, heading, headingChange);
    } 
    
    
  }
  
  return 0;
}
