#ifndef MARCH_H
#define MARCH_H

// void marchingCubes(int dims[3], float *potential, int shift[3], int indexOffset, float *positions, unsigned int *faces, unsigned int &positionIndex, unsigned int &faceIndex);
void marchingCubes2(int dims[3], float *potential, unsigned char *biomes, unsigned char *heightfield, unsigned char *lightfield, float shift[3], float scale[3], float *positions, float *barycentrics, unsigned int &positionIndex, unsigned int &barycentricIndex, unsigned char *skyLights, unsigned char *torchLights);
// void collideBoxEther(int dims[3], float *potential, int shift[3], float *positionSpec, bool &collided, bool &floored, bool &ceiled);

#endif
