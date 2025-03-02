#include "zf_common_headfile.h"
#include "common.h"

/// @brief restrain the value in the range of max and min
/// @param value 
/// @param max 
/// @param min 
void restrictValueI(int32 *value, int32 max, int32 min){
    int32 temp = 0;
    min > max ? (temp = max, max = min, min = temp) : 0;
    *value > max ? *value = max : 0;
    *value < min ? *value = min : 0;
}
/// @brief restrain the value in the range of max and min
/// @param value 
/// @param max 
/// @param min 
void restrictValueF(float *value, float max, float min){
    float temp = 0;
    min > max ? (temp = max, max = min, min = temp) : 0;
    *value > max ? *value = max : 0;
    *value < min ? *value = min : 0;
}