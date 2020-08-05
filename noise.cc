#include "noise.h"
#include <string.h>
#include <iostream>
#include <vector>
#include <array>
#define _USE_MATH_DEFINES
#include <cmath>
#include "biomes.h"

#define PI M_PI
constexpr int FILL_BOTTOM = 1<<0;
constexpr int FILL_TOP = 1<<1;
constexpr int FILL_LEFT = 1<<2;
constexpr int FILL_RIGHT = 1<<3;
constexpr int FILL_FRONT = 1<<4;
constexpr int FILL_BACK = 1<<5;
constexpr int FILL_LOCAL = 1<<6;
constexpr int FILL_BASE = 1<<7;

Noise::Noise(int s, double frequency, int octaves) : fastNoise(s) {
  fastNoise.SetFrequency(frequency);
  fastNoise.SetFractalOctaves(octaves);
}
Noise::Noise(const Noise &noise) : fastNoise(noise.fastNoise) {}
Noise::~Noise() {}
Noise &Noise::operator=(const Noise &noise) {
  fastNoise = noise.fastNoise;
  return *this;
}
double Noise::in2D(float x, float y) {
  return (1.0 + fastNoise.GetSimplexFractal(x, y)) / 2.0;
}
double Noise::in3D(float x, float y, float z) {
  return (1.0 + fastNoise.GetSimplexFractal(x, y, z)) / 2.0;
}

/* void noise2(int seed, double frequency, int octaves, int dims[3], float shifts[3], float offset, float *potential) {
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
} */

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

template <typename T>
inline void _fillOblateSpheroid(float centerX, float centerY, float centerZ, int minX, int minY, int minZ, int maxX, int maxY, int maxZ, float radius, int *dimsP1, T *ether, std::function<T(T, float)> fn) {
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
                const int index = std::floor(lx - minX) +
                  (std::floor(lz - minZ) * dimsP1[0]) +
                  (std::floor(ly - minY) * dimsP1[0] * dimsP1[1]);
                ether[index] = fn(ether[index], distance);
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

float getHeight(int seed, float ax, float ay, float az, float baseHeight, float *freqs, int *octaves, float *scales, float *uvs, float *amps, int limits[3]) {
  float cx = ax - (float)(limits[0])/2.0f;
  float cy = ay - (float)(limits[1])/2.0f;
  float cz = az - (float)(limits[2])/2.0f;

  AxisElevationNoise axisElevationNoise(seed, freqs, octaves);

  std::array<Noise, 3> *elevationNoise;
  float u, v, w;
  if (std::abs(cx) >= std::abs(cy) && std::abs(cx) >= std::abs(cz)) {
    if (cx >= 0) {
      elevationNoise = &axisElevationNoise.noises[0];
      u = az;
      v = ay;
      w = cx;
    } else {
      elevationNoise = &axisElevationNoise.noises[1];
      u = az;
      v = ay;
      w = -cx;
    }
  } else if (std::abs(cy) >= std::abs(cx) && std::abs(cy) >= std::abs(cz)) {
    if (cy >= 0) {
      elevationNoise = &axisElevationNoise.noises[2];
      u = ax;
      v = az;
      w = cy;
    } else {
      elevationNoise = &axisElevationNoise.noises[3];
      u = ax;
      v = az;
      w = -cy;
    }
  } else {
    if (cz >= 0) {
      elevationNoise = &axisElevationNoise.noises[4];
      u = ax;
      v = ay;
      w = cz;
    } else {
      elevationNoise = &axisElevationNoise.noises[5];
      u = ax;
      v = ay;
      w = -cz;
    }
  }

  float height = baseHeight +
    (*elevationNoise)[0].in2D((u + uvs[0]) * scales[0], (v + uvs[0]) * scales[0]) * amps[0] +
    (*elevationNoise)[1].in2D((u + uvs[1]) * scales[1], (v + uvs[1]) * scales[1]) * amps[1] +
    (*elevationNoise)[2].in2D((u + uvs[2]) * scales[2], (v + uvs[2]) * scales[2]) * amps[2];
  return height;
}

void noise3(int seed, float baseHeight, float *freqs, int *octaves, float *scales, float *uvs, float *amps, int dims[3], float shifts[3], int limits[3], float wormRate, float wormRadiusBase, float wormRadiusRate, float objectsRate, float offset, float *potential, unsigned char *heightfield, float *objectPositions, float *objectQuaternions, unsigned int *objectTypes, unsigned int &numObjects, unsigned int maxNumObjects) {
  memset(potential, 0, dims[0]*dims[1]*dims[2]*sizeof(float));
  memset(heightfield, 0, dims[0]*dims[1]*dims[2]*sizeof(unsigned char));
  AxisElevationNoise axisElevationNoise(seed, freqs, octaves);

  // Noise oceanNoise(seed++, 0.001, 4);
  // Noise riverNoise(seed++, 0.001, 4);
  // Noise temperatureNoise(seed++, 0.001, 4);
  // Noise humidityNoise(seed++, 0.001, 4);
  // Noise lavaNoise(seed+5, 0.01, 4);
  Noise nestNoise(seed++, 2, 1);
  Noise nestNoiseX(seed++, 2, 1);
  Noise nestNoiseY(seed++, 2, 1);
  Noise nestNoiseZ(seed++, 2, 1);
  Noise numWormsNoise(seed++, 0.1, 1);
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
  Noise numObjectsNoise(seed++, 10, 1);
  Noise objectsNoiseX(seed++, 10, 1);
  Noise objectsNoiseZ(seed++, 10, 1);
  Noise objectsTypeNoise(seed++, 10, 1);

  int dimsP1[3] = {
    dims[0]+1,
    dims[1]+1,
    dims[2]+1,
  };
  int dimsP3[3] = {
    dims[0]+3,
    dims[1]+3,
    dims[2]+3,
  };

  std::vector<unsigned char> fills(dimsP1[0]*dimsP1[1]*dimsP1[2]);

  for (int x = 0; x < dimsP3[0]; x++) {
    float ax = shifts[0] + x;
    float cx = ax - (float)(limits[0])/2.0f;
    for (int z = 0; z < dimsP3[2]; z++) {
      float az = shifts[2] + z;
      float cz = az - (float)(limits[2])/2.0f;
      for (int y = 0; y < dimsP3[1]; y++) {
        float ay = shifts[1] + y;
        float cy = ay - (float)(limits[1])/2.0f;

        std::array<Noise, 3> *elevationNoise;
        float u, v, w;
        if (std::abs(cx) >= std::abs(cy) && std::abs(cx) >= std::abs(cz)) {
          if (cx >= 0) {
            elevationNoise = &axisElevationNoise.noises[0];
            u = az;
            v = ay;
            w = cx;
          } else {
            elevationNoise = &axisElevationNoise.noises[1];
            u = az;
            v = ay;
            w = -cx;
          }
        } else if (std::abs(cy) >= std::abs(cx) && std::abs(cy) >= std::abs(cz)) {
          if (cy >= 0) {
            elevationNoise = &axisElevationNoise.noises[2];
            u = ax;
            v = az;
            w = cy;
          } else {
            elevationNoise = &axisElevationNoise.noises[3];
            u = ax;
            v = az;
            w = -cy;
          }
        } else {
          if (cz >= 0) {
            elevationNoise = &axisElevationNoise.noises[4];
            u = ax;
            v = ay;
            w = cz;
          } else {
            elevationNoise = &axisElevationNoise.noises[5];
            u = ax;
            v = ay;
            w = -cz;
          }
        }

        float height = baseHeight +
          (*elevationNoise)[0].in2D((u + uvs[0]) * scales[0], (v + uvs[0]) * scales[0]) * amps[0] +
          (*elevationNoise)[1].in2D((u + uvs[1]) * scales[1], (v + uvs[1]) * scales[1]) * amps[1] +
          (*elevationNoise)[2].in2D((u + uvs[2]) * scales[2], (v + uvs[2]) * scales[2]) * amps[2];
        if (x < dimsP1[0] && y < dimsP1[1] && z < dimsP1[2]) {
          int index = x +
            (z * dimsP1[0]) +
            (y * dimsP1[0] * dimsP1[1]);
          potential[index] = (w < height) ? -offset : offset;
          heightfield[index] = (unsigned char)std::min<float>(std::max<float>(8.0f + 0.5f - (height - w), 0.0f), 8.0f);
        }
        if (w < height) {
          int fillIndex = x +
            (z * dimsP3[0]) +
            (y * dimsP3[0] * dimsP3[1]);
          fills[fillIndex] = FILL_LOCAL;
        }
      }
    }
  }

  const int ox = (int)(shifts[0]/dims[0]);
  const int oy = (int)(shifts[1]/dims[1]);
  const int oz = (int)(shifts[2]/dims[2]);
  for (int doz = -4; doz <= 4; doz++) {
    for (int doy = -4; doy <= 4; doy++) {
      for (int dox = -4; dox <= 4; dox++) {
        const int aox = ox + dox;
        const int aoy = oy + doy;
        const int aoz = oz + doz;

        const int nx = aox * dims[0];
        const int ny = aoy * dims[1];
        const int nz = aoz * dims[2];
        const float n = nestNoise.in3D(nx, ny, nz);
        const int numNests = (int)std::floor(n * 2);

        for (int i = 0; i < numNests; i++) {
          const int nx = aox * dims[0] + i * 1000;
          const int ny = aoy * dims[1] + i * 1000;
          const int nz = aoz * dims[2] + i * 1000;
          const float nestX = (float)(aox * dims[0]) + nestNoiseX.in2D(ny, nz) * dims[0];
          const float nestY = (float)(aoy * dims[1]) + nestNoiseY.in2D(nx, nz) * dims[1];
          const float nestZ = (float)(aoz * dims[2]) + nestNoiseZ.in2D(nx, ny) * dims[2];

          const int numWorms = (int)std::floor(numWormsNoise.in3D(nx, ny, nz) * wormRate);
          for (int j = 0; j < numWorms; j++) {
            float cavePosX = nestX;
            float cavePosY = nestY;
            float cavePosZ = nestZ;
            const int caveLength = (int)((0.75 + caveLengthNoise.in3D(nx, ny, nz) * 0.25) * dims[0] * 2);

            float theta = caveThetaNoise.in2D(nx, nz) * PI * 2;
            float deltaTheta = 0;
            float phi = cavePhiNoise.in2D(nx, nz) * PI * 2;
            float deltaPhi = 0;

            const float caveRadius = caveRadiusNoise.in3D(nx, ny, nz);

            for (int len = 0; len < caveLength; len++) {
              const int nx = aox * dims[0] + i + len;
              const int nz = aoz * dims[2] + i + len;

              cavePosX += sin(theta) * cos(phi);
              cavePosY += cos(theta) * cos(phi);
              cavePosZ += sin(phi);

              theta += deltaTheta * 0.2;
              deltaTheta = (deltaTheta * 0.9) + (-0.5 + caveDeltaThetaNoise.in3D(nx, ny, nz));
              phi = phi/2 + deltaPhi/4;
              deltaPhi = (deltaPhi * 0.75) + (-0.5 + caveDeltaPhiNoise.in3D(nx, ny, nz));

              if (caveFillNoise.in3D(nx, ny, nz) >= 0.25) {
                const float centerPosX = cavePosX + (caveCenterNoiseX.in2D(ny, nz) * 4 - 2) * 0.2;
                const float centerPosY = cavePosY + (caveCenterNoiseY.in2D(nx, nz) * 4 - 2) * 0.2;
                const float centerPosZ = cavePosZ + (caveCenterNoiseZ.in2D(nx, ny) * 4 - 2) * 0.2;

                const float radius = wormRadiusBase + wormRadiusRate * caveRadius * sin(len * PI / caveLength);
                _fillOblateSpheroid<float>(centerPosX, centerPosY, centerPosZ, ox * dims[0], oy * dims[1], oz * dims[2], (ox + 1) * dims[0] + 1, (oy + 1) * dims[1] + 1, (oz + 1) * dims[2] + 1, radius, dimsP1, potential, [&](float oldVal, float distance) -> float {
                  return oldVal - (1 + ((radius - std::sqrt(distance)) / radius));
                });
                _fillOblateSpheroid<unsigned char>(centerPosX, centerPosY, centerPosZ, ox * dims[0], oy * dims[1], oz * dims[2], (ox + 1) * dims[0] + 3, (oy + 1) * dims[1] + 3, (oz + 1) * dims[2] + 3, radius, dimsP3, fills.data(), [&](unsigned char oldVal, float distance) -> float {
                  return 0;
                });
              }
            }
          }
        }
      }
    }
  }

  for (int x = 0; x < dims[0]; x++) {
    for (int z = 0; z < dims[2]; z++) {
      for (int y = 0; y < dims[1]; y++) {
        int fillIndex = x +
          (z * dimsP3[0]) +
          (y * dimsP3[0] * dimsP3[1]);
        bool topEmpty = true;
        {
          int dy = 2;
          int ay = y + dy;
          for (int dx = 0; dx < 3; dx++) {
            int ax = x + dx;
            for (int dz = 0; dz < 3; dz++) {
              int az = z + dz;
              int fillIndex = x +
                (z * dimsP3[0]) +
                (y * dimsP3[0] * dimsP3[1]);
              if (fills[fillIndex]&FILL_LOCAL) {
                topEmpty = false;
              }
            }
          }
        }
        if (!topEmpty) {
          fills[fillIndex] |= FILL_TOP;
        }
        bool bottomEmpty = true;
        {
          int dy = 0;
          int ay = y + dy;
          for (int dx = 0; dx < 3; dx++) {
            int ax = x + dx;
            for (int dz = 0; dz < 3; dz++) {
              int az = z + dz;
              int fillIndex = x +
                (z * dimsP3[0]) +
                (y * dimsP3[0] * dimsP3[1]);
              if (fills[fillIndex]&FILL_LOCAL) {
                bottomEmpty = false;
              }
            }
          }
        }
        if (!bottomEmpty) {
          fills[fillIndex] |= FILL_BOTTOM;
        }
        bool leftEmpty = true;
        {
          int dx = 0;
          int ax = x + dx;
          for (int dy = 0; dy < 3; dy++) {
            int ay = y + dy;
            for (int dz = 0; dz < 3; dz++) {
              int az = z + dz;
              int fillIndex = x +
                (z * dimsP3[0]) +
                (y * dimsP3[0] * dimsP3[1]);
              if (fills[fillIndex]&FILL_LOCAL) {
                leftEmpty = false;
              }
            }
          }
        }
        if (!leftEmpty) {
          fills[fillIndex] |= FILL_LEFT;
        }
        bool rightEmpty = true;
        {
          int dx = 2;
          int ax = x + dx;
          for (int dy = 0; dy < 3; dy++) {
            int ay = y + dy;
            for (int dz = 0; dz < 3; dz++) {
              int az = z + dz;
              int fillIndex = x +
                (z * dimsP3[0]) +
                (y * dimsP3[0] * dimsP3[1]);
              if (fills[fillIndex]&FILL_LOCAL) {
                rightEmpty = false;
              }
            }
          }
        }
        if (!rightEmpty) {
          fills[fillIndex] |= FILL_RIGHT;
        }
        bool frontEmpty = true;
        {
          int dz = 2;
          int az = z + dz;
          for (int dx = 0; dx < 3; dx++) {
            int ax = x + dx;
            for (int dz = 0; dz < 3; dz++) {
              int az = z + dz;
              int fillIndex = x +
                (z * dimsP3[0]) +
                (y * dimsP3[0] * dimsP3[1]);
              if (fills[fillIndex]&FILL_LOCAL) {
                frontEmpty = false;
              }
            }
          }
        }
        if (!frontEmpty) {
          fills[fillIndex] |= FILL_FRONT;
        }
        bool backEmpty = true;
        {
          int dz = 0;
          int az = z + dz;
          for (int dx = 0; dx < 3; dx++) {
            int ax = x + dx;
            for (int dy = 0; dy < 3; dy++) {
              int ay = y + dy;
              int fillIndex = x +
                (z * dimsP3[0]) +
                (y * dimsP3[0] * dimsP3[1]);
              if (fills[fillIndex]&FILL_LOCAL) {
                backEmpty = false;
              }
            }
          }
        }
        if (!backEmpty) {
          fills[fillIndex] |= FILL_BACK;
        }

        int fillIndexUpRightForward = (x+1) +
          ((z+1) * dimsP3[0]) +
          ((y+1) * dimsP3[0] * dimsP3[1]);
        if (!(fills[fillIndexUpRightForward]&FILL_LOCAL) && !bottomEmpty) {
          // std::cout << "fill base " << (x+1) << " " << (y+1) << " " << (z+1) << " " << fillIndexUpRightForward << std::endl;
          fills[fillIndexUpRightForward] |= FILL_BASE;
        }
      }
    }
  }

  unsigned int &i = numObjects;
  i = 0;
  for (int x = 0; x < dims[0] && i < maxNumObjects; x++) {
    float ax = shifts[0] + x;
    float cx = ax - (float)(limits[0])/2.0f;
    for (int z = 0; z < dims[2] && i < maxNumObjects; z++) {
      float az = shifts[2] + z;
      float cz = az - (float)(limits[2])/2.0f;
      for (int y = 0; y < dims[1] && i < maxNumObjects; y++) {
        float ay = shifts[1] + y;
        float cy = ay - (float)(limits[1])/2.0f;

        int fillIndex = x +
          (z * dimsP3[0]) +
          (y * dimsP3[0] * dimsP3[1]);
        if ((bool)(fills[fillIndex]&FILL_BASE)) {
          if (numObjectsNoise.in3D(cx, cy, cz) < 0.1) {
            objectPositions[i*3] = ax;
            objectPositions[i*3+1] = ay;
            objectPositions[i*3+2] = az;
            objectQuaternions[i*4] = 0;
            objectQuaternions[i*4+1] = 0;
            objectQuaternions[i*4+2] = 0;
            objectQuaternions[i*4+3] = 1;
            objectTypes[i] = (unsigned int)std::floor(objectsTypeNoise.in3D(cx + i * 1000.0f, cy + i * 1000.0f, cz + i * 1000.0f) * (float)0xFF);
            i++;
            // std::cout << "base yes" << std::endl;
          } /* else {
            std::cout << "no base " << cx << " " << cy << " " << cz << " " << fillIndex << " " << numObjectsNoise.in3D(cx, cy, cz) << std::endl;
          } */
        }
      }
    }
  }
}