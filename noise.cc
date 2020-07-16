#include "noise.h"
#include <string.h>
#include <iostream>
#include <vector>
#include <array>
#define _USE_MATH_DEFINES
#include <cmath>
#include "biomes.h"

#define PI M_PI

Noise::Noise(int s, double frequency, int octaves) : fastNoise(s) {
  fastNoise.SetFrequency(frequency);
  fastNoise.SetFractalOctaves(octaves);
}
Noise::Noise(const Noise &noise) : fastNoise(noise.fastNoise) {}
Noise::~Noise() {}
Noise &Noise::operator=(const Noise &noise) {
  fastNoise = noise.fastNoise;
}
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

inline unsigned char getBiome(float x, float z, Noise &oceanNoise, Noise &riverNoise, Noise &temperatureNoise, Noise &humidityNoise) {
  // const std::pair<int, int> key(x, z);
  // std::unordered_map<std::pair<int, int>, unsigned char>::iterator entryIter = biomeCache.find(key);

  /* if (entryIter != biomeCache.end()) {
    return entryIter->second;
  } else { */
    // unsigned char &biome = biomeCache[key];
    unsigned char biome = 0xFF;
    if (oceanNoise.in2D(x, z) < (80.0 / 255.0)) {
      biome = (unsigned char)BIOME::biOcean;
    }
    if (biome == 0xFF) {
      const double n = riverNoise.in2D(x, z);
      const double range = 0.022;
      if (n > 0.5 - range && n < 0.5 + range) {
        biome = (unsigned char)BIOME::biRiver;
      }
    }
    if (std::pow(temperatureNoise.in2D(x, z), 1.3) < ((4.0 * 16.0) / 255.0)) {
      if (biome == (unsigned char)BIOME::biOcean) {
        biome = (unsigned char)BIOME::biFrozenOcean;
      } else if (biome == (unsigned char)BIOME::biRiver) {
        biome = (unsigned char)BIOME::biFrozenRiver;
      }
    }
    if (biome == 0xFF) {
      const int t = (int)std::floor(std::pow(temperatureNoise.in2D(x, z), 1.3) * 16.0);
      const int h = (int)std::floor(std::pow(humidityNoise.in2D(x, z), 1.3) * 16.0);
      biome = (unsigned char)BIOMES_TEMPERATURE_HUMIDITY[t + 16 * h];
    }

    return biome;
  // }
}
inline float getBiomeHeight(unsigned char b, float x, float z, Noise &elevationNoise1, Noise &elevationNoise2, Noise &elevationNoise3) {
  // const std::tuple<unsigned char, int, int> key(b, x, z);
  // std::unordered_map<std::tuple<unsigned char, int, int>, float>::iterator entryIter = biomeHeightCache.find(key);

  /* if (entryIter != biomeHeightCache.end()) {
    return entryIter->second;
  } else { */
    // float &biomeHeight = biomeHeightCache[key];

    const Biome &biome = BIOMES[b];
    float biomeHeight = std::min<float>(biome.baseHeight +
      elevationNoise1.in2D(x * biome.amps[0][0], z * biome.amps[0][0]) * biome.amps[0][1] +
      elevationNoise2.in2D(x * biome.amps[1][0], z * biome.amps[1][0]) * biome.amps[1][1] +
      elevationNoise3.in2D(x * biome.amps[2][0], z * biome.amps[2][0]) * biome.amps[2][1], 128 - 0.1);
    return biomeHeight;
  // }
}

void _fillOblateSpheroid(float centerX, float centerY, float centerZ, int minX, int minY, int minZ, int maxX, int maxY, int maxZ, float radius, int *dims, float *ether) {
  const int radiusCeil = (int)std::ceil(radius);
  for (int z = -radiusCeil; z <= radiusCeil; z++) {
    const float lz = centerZ + z;
    if (lz >= minZ && lz < maxZ) {
      for (int x = -radiusCeil; x <= radiusCeil; x++) {
        const float lx = centerX + x;
        if (lx >= minX && lx < maxX) {
          for (int y = -radiusCeil; y <= radiusCeil; y++) {
            const float ly = centerY + y;
            if (ly >= minY && ly < maxY) {
              const float distance = x*x + 2 * y*y + z*z;
              if (distance < radius*radius) {
                // const int index = getEtherIndex(std::floor(lx - minX), std::floor(ly), std::floor(lz - minZ));
                const int index = std::floor(lx - minX) +
                  (std::floor(lz - minZ) * dims[0]) +
                  (std::floor(ly - minY) * dims[0] * dims[1]);
                const float distance2 = std::sqrt(distance);
                ether[index] -= 1 + ((radius - distance2) / radius);
              }
            }
          }
        }
      }
    }
  }
}

class AxisElevationNoise {
public:
  AxisElevationNoise(int &seed, float *freqs, int *octaves) {
    noises.reserve(6);
    for (int i = 0; i < 6; i++) {
      noises.push_back(std::array<Noise, 3>{
        Noise(seed++, freqs[0], octaves[0]),
        Noise(seed++, freqs[1], octaves[1]),
        Noise(seed++, freqs[1], octaves[2])
      });
    }
  }
  std::vector<std::array<Noise, 3>> noises;
};

void noise3(int seed, float baseHeight, float *freqs, int *octaves, float *scales, float *uvs, float *amps, int dims[3], float shifts[3], float offset, float *potential) {
  memset(potential, 0, dims[0]*dims[1]*dims[2]*sizeof(float));
  AxisElevationNoise elevationNoise(seed, freqs, octaves);

  // Noise oceanNoise(seed++, 0.001, 4);
  // Noise riverNoise(seed++, 0.001, 4);
  // Noise temperatureNoise(seed++, 0.001, 4);
  // Noise humidityNoise(seed++, 0.001, 4);
  // Noise lavaNoise(seed+5, 0.01, 4);
  Noise nestNoise(seed++, 2, 1);
  Noise nestNoiseX(seed++, 2, 1);
  Noise nestNoiseY(seed++, 2, 1);
  Noise nestNoiseZ(seed++, 2, 1);
  Noise wormNoise(seed++, 2, 1);
  Noise caveLengthNoise(seed++, 2, 1);
  Noise caveRadiusNoise(seed++, 2, 1);
  Noise caveThetaNoise(seed++, 2, 1);
  Noise cavePhiNoise(seed++, 2, 1);
  Noise caveDeltaThetaNoise(seed++, 2, 1);
  Noise caveDeltaPhiNoise(seed++, 2, 1);
  Noise caveFillNoise(seed++, 2, 1);
  Noise caveCenterNoiseX(seed++, 2, 1);
  Noise caveCenterNoiseY(seed++, 2, 1);
  Noise caveCenterNoiseZ(seed++, 2, 1);

  // std::cout << "elevation " << baseHeight << " " << freqs[0] << " " << octaves[0] << " " << amps[0] << " " << amps[0] << std::endl;

  for (int x = 0; x < dims[0]; x++) {
    for (int z = 0; z < dims[2]; z++) {
      float ax = shifts[0] + x;
      float az = shifts[2] + z;

      // std::cout << "get " << ax << " " << az << " " << (elevationNoise1.in2D(ax, az) * amps[0]) << " " << x << " " << z << " : " << shifts[0] << " " << shifts[2] << " " << dims[0] << " " << dims[2] << std::endl;

      float height = // std::min<float>(
        baseHeight +
          elevationNoise.noises[0][0].in2D((ax + uvs[0]) * scales[0], (az + uvs[0]) * scales[0]) * amps[0] +
          elevationNoise.noises[0][1].in2D((ax + uvs[1]) * scales[1], (az + uvs[1]) * scales[1]) * amps[1] +
          elevationNoise.noises[0][2].in2D((ax + uvs[2]) * scales[2], (az + uvs[2]) * scales[2]) * amps[2];/*,
        128 - 0.1
      ); */

      for (int y = 0; y < dims[1]; y++) {
        int index = (x) +
          (z * dims[0]) +
          (y * dims[0] * dims[1]);
        float ay = shifts[1] + y;
        potential[index] = (ay < height) ? -offset : offset;
      }
    }
  }

  constexpr float NUM_CELLS_HEIGHT = 10;
  const int ox = (int)(shifts[0]/dims[0]);
  const int oy = (int)(shifts[1]/dims[1]);
  const int oz = (int)(shifts[2]/dims[2]);
  for (int doz = -4; doz <= 4; doz++) {
    for (int dox = -4; dox <= 4; dox++) {
      const int aox = ox + dox;
      const int aoz = oz + doz;
      const int nx = aox * dims[0];
      const int nz = aoz * dims[2];
      const float n = nestNoise.in2D(nx, nz);
      const int numNests = (int)std::floor(std::max<float>(n * 4, 0));

      for (int i = 0; i < numNests; i++) {
        const int nx = aox * dims[0] + i * 1000;
        const int nz = aoz * dims[2] + i * 1000;
        const float nestX = (float)(aox * dims[0]) + nestNoiseX.in2D(nx, nz) * dims[0];
        const float nestY = nestNoiseY.in2D(nx, nz) * NUM_CELLS_HEIGHT;
        const float nestZ = (float)(aoz * dims[2]) + nestNoiseZ.in2D(nx, nz) * dims[2];

        const int numWorms = 1 + (int)std::floor(std::max<float>(wormNoise.in2D(nx, nz) * 3, 0));
        for (int j = 0; j < numWorms; j++) {
          float cavePosX = nestX;
          float cavePosY = nestY;
          float cavePosZ = nestZ;
          const int caveLength = (int)((0.75 + caveLengthNoise.in2D(nx, nz) * 0.25) * dims[0] * 2);

          float theta = caveThetaNoise.in2D(nx, nz) * PI * 2;
          float deltaTheta = 0;
          float phi = cavePhiNoise.in2D(nx, nz) * PI * 2;
          float deltaPhi = 0;

          const float caveRadius = caveRadiusNoise.in2D(nx, nz);

          for (int len = 0; len < caveLength; len++) {
            const int nx = aox * dims[0] + i + len;
            const int nz = aoz * dims[2] + i + len;

            cavePosX += sin(theta) * cos(phi);
            cavePosY += cos(theta) * cos(phi);
            cavePosZ += sin(phi);

            theta += deltaTheta * 0.2;
            deltaTheta = (deltaTheta * 0.9) + (-0.5 + caveDeltaThetaNoise.in2D(nx, nz));
            phi = phi/2 + deltaPhi/4;
            deltaPhi = (deltaPhi * 0.75) + (-0.5 + caveDeltaPhiNoise.in2D(nx, nz));

            if (caveFillNoise.in2D(nx, nz) >= 0.25) {
              const float centerPosX = cavePosX + (caveCenterNoiseX.in2D(nx, nz) * 4 - 2) * 0.2;
              const float centerPosY = cavePosY + (caveCenterNoiseY.in2D(nx, nz) * 4 - 2) * 0.2;
              const float centerPosZ = cavePosZ + (caveCenterNoiseZ.in2D(nx, nz) * 4 - 2) * 0.2;

              const float radius = 2 + 3.5 * caveRadius * sin(len * PI / caveLength);
              _fillOblateSpheroid(centerPosX, centerPosY, centerPosZ, ox * dims[0], oy * dims[1], oz * dims[2], (ox + 1) * dims[0], (oy + 1) * dims[1], (oz + 1) * dims[2], radius, dims, potential);
            }
          }
        }
      }
    }
  }

  if (shifts[1] < 1) {
    const int y = 0;
    for (int x = 0; x < dims[0]; x++) {
      for (int z = 0; z < dims[2]; z++) {
        int index = (x) +
          (z * dims[0]) +
          (y * dims[0] * dims[1]);
        potential[index] = -offset;
      }
    }
  }
}