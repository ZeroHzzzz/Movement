#include "my_filter.h"
/// @brief low pass filter
/// @param output 
/// @param lastOutput 
/// @param alpha 
inline void lowPassFilterF(float *output, float *lastOutput, float alpha){
    *output = (1 - alpha) * (*output) + (alpha) * (*lastOutput);
}

inline void lowPassFilterI(int32 *output, int32 *lastOutput, float alpha){
    *output = (1 - alpha) * (float)(*output) + (alpha) * (float)(*lastOutput);
}