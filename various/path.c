#include <stdio.h>

#include <math.h>

void output(float x, float y, float str, float param) {
  printf("{ %05.1ff, %05.1ff, %1.1ff, %06.1ff },\n", (x+1)*75, (y+1)*75, str, param);
}

void searchat(float x, float y) {
  output(x, y, 0.4, 500);
  //output(x+0.02, y, 0.4, 1000);
  //output(x, y+0.02, 0.4, 1000);
  //output(x-0.02, y, 0.4, 1000);
  //output(x, y-0.02, 0.4, 1000);
}

void pickupto(float x, float y) {
  output(x, y, 0.4, 0);
  //output(x, y, 0.4, 1000);
  output(x, y, 0, 1000);
}

void moveto(float x, float y) {
  output(x, y, 0, 0);
}

#define POINTS 15
#define TWOPI (3.1415*2.0)

int main(int argc, char** argv) {
  moveto(-1,-1);
  
  for (int j=0; j<2; j++)
  for (int i=0; i<POINTS; i++) {
    float angle = ((float)i/POINTS + 0.5 + 0.125) * TWOPI ;
    if (!(i%2) != !j)
      pickupto(cos(angle)*0.8, sin(angle)*0.8);
    else {
      moveto(cos(angle)*0.8, sin(angle)*0.8);
      searchat(cos(angle)*0.8, sin(angle)*0.8);
    }
  }
    
  
  moveto(-1,-1);
  
}
