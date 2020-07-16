#ifndef NOISE_H
#define NOISE_H

#include "FastNoise.h"

class Noise {
 public:
  FastNoise fastNoise;

  explicit Noise(int s = 0, double frequency = 0.01, int octaves = 1);
  Noise(const Noise &noise);
  ~Noise();
  Noise &operator=(const Noise &noise);

  double in2D(float x, float y);
  double in3D(float x, float y, float z);
};

void noise2(int seed, double frequency, int octaves, int dims[3], float shifts[3], float offset, float *potential);
void noise3(int seed, float baseHeight, float *freqs, int *octaves, float *scales, float *uvs, float *amps, int dims[3], float shifts[3], int limits[3], float wormRate, float wormRadiusBase, float wormRadiusRate, float offset, float *potential);

#endif
