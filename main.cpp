#include "main.hpp"
#include "sdl.hpp"

#include <unistd.h>
#include <webots/utils/Motion.hpp>

#include <SDL.h>
#include <SDL_image.h>

#define G 9.81

#define TIME_RATE 0.1

NaoRobot *NaoRobot::nao_robot = nullptr;

/* Utility function */
double clamp(double value, double min, double max) {
  return (value < min) ? min : (value > max) ? max : value;
}

int main() {
  LibXR::PlatformInit();

  Sim2D sim2d;

  while (true) {
    sleep(1);
  }

  return 0;
}
