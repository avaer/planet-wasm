#include "noise.h"
#include <string.h>

Noise::Noise(int s, double frequency, int octaves) : fastNoise(s) {
  fastNoise.SetFrequency(frequency);
  fastNoise.SetFractalOctaves(octaves);
}

Noise::~Noise() {}

double Noise::in2D(double x, double y) {
  return (1.0 + fastNoise.GetSimplexFractal(x, y)) / 2.0;
}
double Noise::in3D(double x, double y, double z) {
  return (1.0 + fastNoise.GetSimplexFractal(x, y, z)) / 2.0;
}

void noise2(int seed, double frequency, int octaves, int dims[3], int border, float offset, float *potential) {
  memset(potential, 0, dims[0]*dims[1]*dims[2]*sizeof(float));
  Noise noise(seed, frequency, octaves);
  for (int x = border; x < dims[0] - border; x++) {
    for (int y = border; y < dims[1] - border; y++) {
      for (int z = border; z < dims[2] - border; z++) {
        int index = (x) +
          (z * dims[0]) +
          (y * dims[0] * dims[1]);
        potential[index] = offset + noise.in3D(x, y, z);
      }
    }
  }
}