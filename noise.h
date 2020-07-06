#ifndef NOISE_H
#define NOISE_H

#include "FastNoise.h"

class Noise {
 public:
  FastNoise fastNoise;

  explicit Noise(int s = 0, double frequency = 0.01, int octaves = 1);
  ~Noise();

  double in2D(double x, double y);
  double in3D(double x, double y, double z);
};

void noise2(int seed, double frequency, int octaves, int dims[3], int border, float offset, float *potential);

#endif