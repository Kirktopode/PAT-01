#include "JVector.h"
#include <stdio.h>

int main(){
  JVector test(1, 2, 3);
  test.print();
  test = test + test;
  test.print();
  test = test * 3;
  test.print();
  test = test / 2;
  test.print();
  float ftest = test.dot(test);
  
  JVector test2(3, 4, 0);
  float ftest2 = test2.norm();
  
  test2.print();
  printf("%.2f %.2f\n", ftest, ftest2);
  
  test2 = test.cross(test2);
  test2.print();
}
