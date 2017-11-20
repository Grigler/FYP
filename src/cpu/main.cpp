#include <glm/glm.hpp>

#include <chrono>

#include "RB.h"

//Converting shitty clock stuff to high-resolution second counter
#define TIME_S(T) (std::chrono::duration_cast<std::chrono::microseconds>(T).count() / 1000000.0f)

int main()
{
  //Timer stuff
  std::chrono::high_resolution_clock clock;
  auto t = clock.now();
  float dt = 0.0f;

  //RB init
  RB rb;
  rb.pos = glm::vec3(0, 0, 0);
  rb.ApplyForceAtLocation(glm::vec3(0, 1, 0), glm::vec3(5, 0, 5));
  //rb.ApplyForceAtLocation(glm::vec3(0, -100, 0), glm::vec3(0, 0, -5));
  int epoch = 0;

  FILE *file = fopen("dump.csv", "w");
  fprintf(file, "Epoch,Pos Y,Angle\n");
  while (epoch < 2500)
  {
    //DEBUG
    //_sleep(16);

    //RB stuff here
    rb.Integrate(0.016);
    //DEBUG
    //printf("P: %.2f, %.2f, %.2f\n", rb.pos.x, rb.pos.y, rb.pos.z);
    //printf("Angle: %f\n\n", glm::degrees(glm::angle(rb.rot)));

    fprintf(file, "%i,%.4f,%.4f\n", epoch, rb.pos.y, glm::degrees(glm::angle(rb.rot)));

    //DT handling
    dt = TIME_S(clock.now() - t);
    t = clock.now();
    epoch++;
  }
  fclose(file);

  return 0;
}