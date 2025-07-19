/* main.cpp */
#include "movement_detection.hpp"

int main() {
  raydar::detect_objects("data/f-35/metadata.json", 2.0f);

  return 0;
}