#include <stdlib.h>
#include <iostream>
#include "ap_fixed.h"

#define INPUT_LENGTH (10000)
#define ITERATIONS (10)
typedef ap_fixed<16, 8> fixed_t;

extern "C"
{
  // void nBodySimulation2D(float *particles);
  void krnl_nbody(float *particles, float *temp, int iterations);
}
