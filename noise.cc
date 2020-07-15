#include "noise.h"
#include <string.h>
#include <iostream>

Noise::Noise(int s, double frequency, int octaves) : fastNoise(s) {
  fastNoise.SetFrequency(frequency);
  fastNoise.SetFractalOctaves(octaves);
}

Noise::~Noise() {}

double Noise::in2D(float x, float y) {
  return (1.0 + fastNoise.GetSimplexFractal(x, y)) / 2.0;
}
double Noise::in3D(float x, float y, float z) {
  return (1.0 + fastNoise.GetSimplexFractal(x, y, z)) / 2.0;
}

void noise2(int seed, double frequency, int octaves, int dims[3], float shifts[3], float offset, float *potential) {
  memset(potential, 0, dims[0]*dims[1]*dims[2]*sizeof(float));
  Noise noise(seed, frequency, octaves);
  for (int x = 0; x < dims[0]; x++) {
    for (int y = 0; y < dims[1]; y++) {
      for (int z = 0; z < dims[2]; z++) {
        int index = (x) +
          (z * dims[0]) +
          (y * dims[0] * dims[1]);
        potential[index] = offset + noise.in3D(x + shifts[0], y + shifts[1], z + shifts[2]);
      }
    }
  }
}

void noise3(int seed, double frequency, int octaves, float heightScale, int dims[3], float shifts[3], float offset, float *potential) {
  memset(potential, 0, dims[0]*dims[1]*dims[2]*sizeof(float));
  Noise noise(seed, frequency, frequency);
  for (int x = 0; x < dims[0]; x++) {
    for (int z = 0; z < dims[2]; z++) {
      const float height = noise.in2D(x + shifts[0], z + shifts[2]) * heightScale;
      for (int y = 0; y < dims[1]; y++) {
        int index = (x) +
          (z * dims[0]) +
          (y * dims[0] * dims[1]);
        potential[index] = (y + shifts[1]) < height ? -offset : offset;
      }
    }
  }
}