// #ifndef _FUZZY_H
// #define _FUZZY_H
// #include "zf_common_headfile.h"
// #include "pid.h"
// typedef enum{
//     NB,
//     NM,
//     NS,
//     ZO,
//     PS,
//     PM,
//     PB,
// }FuzzySetType;
// // extern FuzzySetType fuzzyRule[7][7];
// typedef struct{
//     float begin;
//     float end;
// }Interval;
// typedef struct{
//     float kp;
//     float ki;
//     float kd;
// }pidPara;
// typedef struct{
//     float e;
//     float ec;
//     float eLast;
//     int8 eFuzzySet;
//     int8 ecFuzzySet;
// }FuzzyVariable;
// typedef struct{
//     float e;
//     int8 eFuzzySet;
// }SimpleFuzzyVariable;
// typedef struct{
//     Interval inputInterval[7];
//     float outputKpSet[7];
//     float outputKiSet[7];
//     float outputKdSet[7];
//     float memberShip[4];

//     FuzzyVariable input;
//     pidPara output;
// }FuzzyPID;
// typedef struct{
//     Interval inputInterval[6];
//     float outputKpSet[7];
//     float outputKiSet[7];
//     float outputKdSet[7];
//     float memberShip;
//     SimpleFuzzyVariable input;
//     pidPara output;
// }SimpleFuzzyPID;
// void fuzzySetInit(FuzzyPID *pid, float inputSet[7], float outputKpSet[7], float outputKiSet[7], float outputKdSet[7]);
// void fuzzyProcess(FuzzyPID *pid, float ref, float set, pid_type_def *pidOut);
// void simpleFuzzySetInit(SimpleFuzzyPID *pid, float inputSet[7], float outputKpSet[7], float outputKiSet[7], float outputKdSet[7]);
// void simpleFuzzyProcess(SimpleFuzzyPID *pid, float ref, float set, pid_type_def *pidOut);
// #endif