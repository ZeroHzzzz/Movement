
#include "zf_common_headfile.h"
#define noiseFilter(x,y) (fabsf(x)<y ? 0:x )
//low pass filter
//alpha: usually 0.1~0.2
void lowPassFilterF(float *output, float *lastOutput, float alpha);
void lowPassFilterI(int32 *output, int32 *lastOutput, float alpha);